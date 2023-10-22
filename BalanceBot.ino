
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

// Objects
Adafruit_NeoPixel pixel{1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ400};

odrv::ImuWrapper imu;

ODriveArduinoCAN left_motor{0};   // Node ID 0
ODriveArduinoCAN right_motor{1};  // Node ID 1

CANSAME5x CAN;

odrv::BalanceControl balancer;

// Constants
const float kControlLoopPeriod = 0.01f;  // [sec]

const float kWheelDiameter = 0.1524f;  // [m] Wheel Diameter
const float kTrackWidth    = 1.0f;     // [m] Distance between wheels
const float kJ             = 1.0f;     // [Nm/(rad/s^2)] Rotational inertia

struct CmdPair {
    float drive;
    float steer;
};

enum class State {
    Idle,
    Active,
    Error
};

const std::array<const char*, 3> StateStrs = {"Idle", "Active", "Error"};

void setup() {
    // Wait for BNO 085 to boot
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
    // while (!Serial) {
    //     delay(1);
    // }

    Serial.println("Connected to serial at 115200 baud.");

    // Start CAN at 500kbps
    CAN.begin(500000);
    CAN.onReceive(can_onReceive);

    Serial.println("Connected to CAN at 500kbps");

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
    balancer.settings.vel_controller.Kp         = 10.0f;
    balancer.settings.vel_controller.Ki         = 1.0f;
    balancer.settings.vel_controller.iterm_min  = -10.0f;  // [deg]
    balancer.settings.vel_controller.output_min = -10.0f;  // [deg]
    balancer.settings.vel_controller.iterm_max  = 10.0f;   // [deg]
    balancer.settings.vel_controller.output_max = 10.0f;   // [deg]

    balancer.settings.pitch_controller.Kp         = 10.0f;
    balancer.settings.pitch_controller.Ki         = 10.0f;
    balancer.settings.pitch_controller.iterm_min  = -100.0f;  // [deg/s]
    balancer.settings.pitch_controller.iterm_max  = 100.0f;   // [deg/s]
    balancer.settings.pitch_controller.output_min = -100.0f;  // [deg/s]
    balancer.settings.pitch_controller.output_max = 100.0f;   // [deg/s]

    balancer.settings.pitch_rate_controller.Kp         = 0.1f;
    balancer.settings.pitch_rate_controller.Ki         = 0.0f;
    balancer.settings.pitch_rate_controller.output_min = -10.0f;  // [Nm]
    balancer.settings.pitch_rate_controller.output_max = 10.0f;   // [Nm]

    balancer.settings.J  = 1.0f;
    balancer.settings.Ts = 0.01f;  // 10ms control loop;
}

void loop() {
    uint32_t start = micros();

    static State state = State::Idle;

    // Blink the LED at 1Hz
    odrv::blink(1000);

    // Read IMU
    imu.read();

    // We run the control loop when there's data available from the IMU (100Hz)
    static odrv::Timer loop_timer(10);  // 10ms control loop
    if (loop_timer.isExpired()) {
        loop_timer.reset();

        // State Machine
        state = run_state_machine(state);

        // TODO:  Reset rx_lpf on disable
        CmdPair rx     = getControllerCmds();
        CmdPair rx_lpf = filterCmds(rx);

        float drive_torque = driveControl(rx_lpf.drive, state == State::Active);
        float steer_torque = steerControl(rx_lpf.steer);

        right_motor.set_input_torque_msg.Input_Torque = +0.5f * (drive_torque + steer_torque);
        left_motor.set_input_torque_msg.Input_Torque  = -0.5f * (drive_torque - steer_torque);

        if (state != State::Active) {
            right_motor.set_input_torque_msg.Input_Torque = 0.0f;
            left_motor.set_input_torque_msg.Input_Torque  = 0.0f;
        }

        sendCyclic();

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

float driveControl(const float vel_target, bool enable) {
    // Pitch angle and rate from IMU

    // Motor speeds
    const float vel_right = right_motor.get_encoder_estimates_msg.Vel_Estimate * (kWheelDiameter * kPi);         // [m/s] right wheel speed
    const float vel_left  = -1.0f * left_motor.get_encoder_estimates_msg.Vel_Estimate * (kWheelDiameter * kPi);  // [m/s] left wheel speed

    // TODO:  Verify yaw rate calculation matches gyro reading
    const float vel_actual = (vel_right + vel_left) / 2.0f;  //  - imu.pitch_rate;  // [m/s] Vehicle speed
    // const float yaw_rate   = (vel_right - vel_left) / (2.0f * kTrackWidth);  // [rad/s] Vehicle yaw Rate, per motor sensors
    // const float gyro_yaw   = imu.yaw;                                        // [rad/s] Vehicle yaw rate, per gyro

    balancer.inputs.enable     = enable;
    balancer.inputs.vel_target = vel_target;

    balancer.inputs.pitch      = imu.pitch;
    balancer.inputs.pitch_rate = imu.pitch_rate;
    balancer.inputs.vel        = (vel_right + vel_left) / 2.0f;

    balancer.update();

    return balancer.getOutputs().torque_cmd;
}

float steerControl(const float yaw_target) {
    return 0.0f;
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

void sendCyclic() {
    sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetControllerModeMsg));
    sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetControllerModeMsg));

    sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetLimitsMsg));
    sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetLimitsMsg));

    sendCanMsg(left_motor.encode(ODriveArduinoCAN::kSetInputTorqueMsg));
    sendCanMsg(right_motor.encode(ODriveArduinoCAN::kSetInputTorqueMsg));
}

CmdPair filterCmds(const CmdPair& rx) {
    // Lowpass filters
    static odrv::lpf drive_lpf{kControlLoopPeriod, 0.5f};
    static odrv::lpf steer_lpf{kControlLoopPeriod, 1.0f};

    return {
        drive_lpf.step(rx.drive),
        steer_lpf.step(rx.steer),
    };
}

CmdPair getControllerCmds() {
    // Read RC controller
    return {};
}
