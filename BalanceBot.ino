
// Uno R4
// #include <Arduino_CAN.h>

// Adafruit Feather M4 CAN
#include <Adafruit_NeoPixel.h>
#include <CANSAME5x.h>

#include <array>

#include "ODriveEnums.h"
#include "balancer.hpp"
#include "can_simple/can_helpers.hpp"
#include "can_simple/can_simple_messages.hpp"
#include "utils.hpp"

#define DEBUG

// On-board NeoPixel object
Adafruit_NeoPixel pixel{1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ400};

// Feather M4 CAN interface
CANSAME5x CAN;

// Convenience wrapper for the IMU
odrv::ImuWrapper imu;

// Controllers used for balancing and steering
odrv::BalanceControl balance_control;
odrv::PIDController  steer_control;

// CAN communication objects for the ODrives
ODriveArduinoCAN left_motor{0};   // Node ID 0
ODriveArduinoCAN right_motor{1};  // Node ID 1

// Constants
constexpr float kControlLoopPeriod = 0.01f;  // [sec]

// Vehicle parameters
constexpr float kWheelDiameter = 0.1524f;  // [m] Wheel Diameter
constexpr float kTrackWidth    = 1.0f;     // [m] Distance between wheels
constexpr float kJ             = 1.0f;     // [Nm/(rad/s^2)] Rotational inertia

struct CmdPair {
    float drive;
    float steer;
};

enum class State {
    Idle,
    Active,
    Error
};

// Name lookup for State
constexpr std::array<const char*, 3> StateStrs = {"Idle", "Active", "Error"};

void setup() {
    // Wait for BNO085 to boot
    delay(100);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_CAN_STANDBY, OUTPUT);
    pinMode(PIN_CAN_BOOSTEN, OUTPUT);
    pinMode(11, OUTPUT);

    // Enable CAN transceiver
    digitalWrite(PIN_CAN_STANDBY, false);  // turn off STANDBY
    digitalWrite(PIN_CAN_BOOSTEN, true);   // turn on booster
    digitalWrite(11, LOW);

    Serial.begin(115200);
    Serial.println("Connected to serial at 115200 baud.");

    // Start CAN at 500kbps
    CAN.begin(500000);
    CAN.onReceive(can_onReceive);

    Serial.println("Connected to CAN at 500kbps");

    // Start the IMU on i2c
    imu.begin();

    Serial.println("Connected to IMU at 400kHz");
    Serial.println("Vel,Pcmd,Pitch,RateCmd,Rate,TorqueCmd");

    // Setup our desired control and input modes
    left_motor.set_controller_mode_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
    left_motor.set_controller_mode_msg.Input_Mode   = INPUT_MODE_PASSTHROUGH;

    right_motor.set_controller_mode_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
    right_motor.set_controller_mode_msg.Input_Mode   = INPUT_MODE_PASSTHROUGH;

    // Setup the velocity and current limits that we'll use
    left_motor.set_limits_msg.Velocity_Limit = 1000.0f;
    left_motor.set_limits_msg.Current_Limit  = 40.0f;

    right_motor.set_limits_msg.Velocity_Limit = 1000.0f;
    right_motor.set_limits_msg.Current_Limit  = 40.0f;

    // Balancing controller settings
    balance_control.settings.vel_controller.Kp         = 10.0f;
    balance_control.settings.vel_controller.Ki         = 1.0f;
    balance_control.settings.vel_controller.iterm_min  = -10.0f;  // [deg]
    balance_control.settings.vel_controller.output_min = -10.0f;  // [deg]
    balance_control.settings.vel_controller.iterm_max  = 10.0f;   // [deg]
    balance_control.settings.vel_controller.output_max = 10.0f;   // [deg]

    balance_control.settings.pitch_controller.Kp         = 10.0f;
    balance_control.settings.pitch_controller.Ki         = 10.0f;
    balance_control.settings.pitch_controller.iterm_min  = -100.0f;  // [deg/s]
    balance_control.settings.pitch_controller.iterm_max  = 100.0f;   // [deg/s]
    balance_control.settings.pitch_controller.output_min = -100.0f;  // [deg/s]
    balance_control.settings.pitch_controller.output_max = 100.0f;   // [deg/s]

    balance_control.settings.pitch_rate_controller.Kp         = 0.1f;
    balance_control.settings.pitch_rate_controller.Ki         = 0.0f;
    balance_control.settings.pitch_rate_controller.output_min = -10.0f;  // [Nm]
    balance_control.settings.pitch_rate_controller.output_max = 10.0f;   // [Nm]

    balance_control.settings.J  = 1.0f;
    balance_control.settings.Ts = kControlLoopPeriod;  // 10ms control loop;

    // // Yaw control settings
    // steer_control.settings.Kp         = 10.0f;
    // steer_control.settings.Ki         = 10.0f;
    // steer_control.settings.iterm_min  = -100.0f;  // [Nm]
    // steer_control.settings.iterm_max  = 100.0f;   // [Nm]
    // steer_control.settings.output_min = -100.0f;  // [Nm]
    // steer_control.settings.output_max = 100.0f;   // [Nm]
}

void loop() {
    static odrv::Timer control_timer(10);
    static State       state = State::Idle;

    uint32_t start = micros();

    // Blink the LED at 1Hz
    odrv::blink(1000);

    // Read IMU
    imu.read();

    // Runs the control loop at 100Hz
    if (control_timer.isExpired()) {
        control_timer.reset();

        // Run State Machine
        state = run_state_machine(state);

        // Only run our controllers if we're in the active state
        const bool enable = (state == State::Active);

        // Get forward and left/right commands from joystick
        CmdPair rx = getControllerCmds(enable);

        // Yaw-priority ISO pattern mixer (maintains 100% wheel speed differential at 100% joystick steer position)
        const float steer_cmd = rx.steer * 0.5f;
        const float drive_cmd = rx.drive * (1.0f - abs(steer_cmd));

        // Motor speeds
        const float vel_right = right_motor.get_encoder_estimates_msg.Vel_Estimate * (kWheelDiameter * kPi);         // [m/s] right wheel speed
        const float vel_left  = -1.0f * left_motor.get_encoder_estimates_msg.Vel_Estimate * (kWheelDiameter * kPi);  // [m/s] left wheel speed

        // TODO:  Verify yaw rate calculation matches gyro reading
        const float vel_actual = (vel_right + vel_left) / 2.0f;                  // [m/s] Vehicle speed
        const float yaw_rate   = (vel_right - vel_left) / (2.0f * kTrackWidth);  // [rad/s] Vehicle yaw rate, per motor sensors

        // Run balancing and steering controllers
        const float drive_torque = balance_control.update(enable, drive_cmd, imu.pitch, imu.pitch_rate, vel_actual);
        const float steer_torque = steer_control.update(enable, steer_cmd, yaw_rate, kControlLoopPeriod);

        // Convert from drive/steer to left/right motor torques
        right_motor.set_input_torque_msg.Input_Torque = +0.5f * (drive_torque + steer_torque);
        left_motor.set_input_torque_msg.Input_Torque  = -0.5f * (drive_torque - steer_torque);

        // Set commands to 0 if we're not in the active state
        if (state != State::Active) {
            right_motor.set_input_torque_msg.Input_Torque = 0.0f;
            left_motor.set_input_torque_msg.Input_Torque  = 0.0f;
        }

        // Send the 100hz CAN messages
        sendCAN();

        uint32_t end = micros();

#ifdef DEBUG
        // if ((size_t)state < StateStrs.size()) {
        //     Serial.println(StateStrs[(size_t)state]);
        // }

        // Serial.printf("States L: %s\n", ODriveStateStrings[left_motor.heartbeat_msg.Axis_State]);
        // Serial.printf("States R: %s\n", ODriveStateStrings[right_motor.heartbeat_msg.Axis_State]);
        // Serial.printf("Quat: w: % 06.2f, i: % 06.2f, j: % 06.2f, k: % 06.2f\n", imu.qw, imu.qi, imu.qj, imu.qk);
        Serial.printf("Angles R: % 06.2f P: % 06.2f Y: % 06.2f\n", imu.roll, imu.pitch, imu.yaw);
        // Serial.printf("Rates  R: % 06.2f P: % 06.2f Y: % 06.2f\n", imu.roll_rate, imu.pitch_rate, imu.yaw_rate);
        // Serial.printf("Speeds L: % 06.2f R: % 06.2f\n", -1.0f * left_motor.get_encoder_estimates_msg.Vel_Estimate, right_motor.get_encoder_estimates_msg.Vel_Estimate);
        // Serial.printf("%dus %3.1f%%\n", end - start, (end - start) / 100.0f);
        // Serial.println();
#endif
    }
}

State run_state_machine(State state) {
    static odrv::Timer vertical_timer{2000};  // 2 sec timer for vertical

    State next_state = state;

    switch (state) {
        case State::Idle: {
            // If pitch is within 5 degrees for 2 seconds, enable motors
            if (fabsf(imu.pitch) > 5.0f) {
                vertical_timer.reset();
            } else if (vertical_timer.isExpired() && (!imu.getIsTimedOut())) {
                setAxisStates(AXIS_STATE_CLOSED_LOOP_CONTROL);
                next_state = State::Active;
            }

        } break;

        case State::Active: {
            // Check for errors
            bool pitch_over  = fabsf(imu.pitch) > 20.0f;
            bool imu_timeout = imu.getIsTimedOut();

            bool left_error  = left_motor.heartbeat_msg.Axis_Error != 0;
            bool right_error = right_motor.heartbeat_msg.Axis_Error != 0;

            if (pitch_over || imu_timeout || left_error || right_error) {
                vertical_timer.reset();

                setAxisStates(AXIS_STATE_IDLE);
                next_state = State::Idle;
            }

        } break;

        case State::Error:
        default: {
            setAxisStates(AXIS_STATE_IDLE);
        } break;
    }

    return next_state;
}

void can_onReceive(int packetSize) {
    can_Message_t rxmsg;
    rxmsg.id = CAN.packetId();
    CAN.readBytes(rxmsg.data, 8);

    switch (ODriveArduinoCAN::get_node_id(rxmsg.id)) {
        case 0: left_motor.decode(rxmsg); break;
        case 1: right_motor.decode(rxmsg); break;
        default: break;
    }
}

void sendCanMsg(const can_Message_t& msg) {
    CAN.beginPacket(msg.id);
    CAN.write(msg.data, msg.len);
    CAN.endPacket();
}

void setAxisStates(ODriveAxisState state) {
    left_motor.set_axis_state_msg.Axis_Requested_State  = state;
    right_motor.set_axis_state_msg.Axis_Requested_State = state;

    sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetAxisStateMsg));
    sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetAxisStateMsg));
}

void sendCAN() {
    sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetControllerModeMsg));
    sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetControllerModeMsg));

    sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetLimitsMsg));
    sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetLimitsMsg));

    sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetInputTorqueMsg));
    sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetInputTorqueMsg));
}

CmdPair filterCmds(const CmdPair& rx) {
    // Lowpass filters
    static odrv::lpf drive_lpf{kControlLoopPeriod, 1.0f};
    static odrv::lpf steer_lpf{kControlLoopPeriod, 1.0f};

    return {
        drive_lpf.step(rx.drive),
        steer_lpf.step(rx.steer),
    };
}

CmdPair getControllerCmds(bool enable) {
    // TODO:  Read controller
    CmdPair rx{};
    return filterCmds(rx);
}
