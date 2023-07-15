
// Uno R4
// #include <Arduino_CAN.h>

// Adafruit Feather M4 CAN
#include <CANSAME5x.h>

#include <array>

#include "ODriveEnums.h"
#include "can_simple/can_helpers.hpp"
#include "can_simple/can_simple_messages.hpp"
#include "utils.hpp"

#define DEBUG

// Objects
odrv::ImuWrapper imu;
ODriveCAN left_motor{0};   // Node ID 0
ODriveCAN right_motor{1};  // Node ID 1

CANSAME5x CAN;

odrv::PIControllerClass vel_controller;
odrv::PIControllerClass pitch_controller;
odrv::PIControllerClass pitch_rate_controller;

// Constants
constexpr float kControlLoopPeriod = 0.01f;  // [sec]
constexpr float kRadToDeg          = 360.0 / TWO_PI;

constexpr float kWheelDiameter = 0.1524f;  // [m] Wheel Diameter
constexpr float kTrackWidth    = 1.0f;     // [m] Distance between wheels
constexpr float kJ             = 1.0f;     // [Nm/(rad/s^2)] Rotational inertia

struct CmdPair {
    float drive;
    float steer;
};

void setup() {
    // Wait for BNO 085 to boot
    delay(100);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_CAN_STANDBY, OUTPUT);
    pinMode(PIN_CAN_BOOSTEN, OUTPUT);

    // Enable CAN transceiver
    digitalWrite(PIN_CAN_STANDBY, false);  // turn off STANDBY
    digitalWrite(PIN_CAN_BOOSTEN, true);   // turn on booster

    Serial.begin(115200);
    Serial.println(F("Connected to serial at 115200 baud."));

    // Start CAN at 500kbps
    CAN.begin(500000);
    CAN.onReceive(can_onReceive);

    Serial.println(F("Connected to CAN at 500kbps"));

    imu.begin();
}

enum class State {
    Idle,
    Active,
    Error
};

constexpr std::array<const char*, 3> StateStrs = {"Idle", "Active", "Error"};

void loop() {
    uint32_t start = micros();
    static odrv::Timer loop_timer(10);        // 10ms control loop
    static odrv::Timer vertical_timer{2000};  // 2 sec timer for vertical

    static State state;
    static State next_state;

    // Blink the LED at 1Hz
    blink(1000);

    // We run the control loop when there's data available from the IMU (100Hz)
    if (loop_timer.isExpired()) {
        loop_timer.reset();

        state = next_state;

        // Read IMU
        imu.step();

        switch (state) {
            case State::Idle: {
                left_motor.set_axis_state_msg.Axis_Requested_State  = AXIS_STATE_IDLE;
                right_motor.set_axis_state_msg.Axis_Requested_State = AXIS_STATE_IDLE;

                left_motor.set_input_torque_msg.Input_Torque  = 0.0f;
                right_motor.set_input_torque_msg.Input_Torque = 0.0f;

                // If pitch is within 5 degrees for 2 seconds, enable motors
                if (fabsf(imu.pitch) > 5.0f) {
                    vertical_timer.reset();
                } else if (vertical_timer.isExpired()) {
                    next_state = State::Active;

                    left_motor.set_axis_state_msg.Axis_Requested_State  = AXIS_STATE_CLOSED_LOOP_CONTROL;
                    right_motor.set_axis_state_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;

                    left_motor.set_controller_mode_msg.Control_Mode  = CONTROL_MODE_TORQUE_CONTROL;
                    right_motor.set_controller_mode_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
                    left_motor.set_controller_mode_msg.Input_Mode    = INPUT_MODE_PASSTHROUGH;
                    right_motor.set_controller_mode_msg.Input_Mode   = INPUT_MODE_PASSTHROUGH;

                    can_sendMsg(left_motor.encode(ODriveCAN::kSetAxisStateMsg));
                    can_sendMsg(right_motor.encode(ODriveCAN::kSetAxisStateMsg));

                    can_sendMsg(left_motor.encode(ODriveCAN::kSetControllerModeMsg));
                    can_sendMsg(right_motor.encode(ODriveCAN::kSetControllerModeMsg));
                }

            } break;

            case State::Active: {
                next_state = State::Active;

                // TODO:  Reset rx_lpf on disable
                const CmdPair rx     = getControllerCmds();
                const CmdPair rx_lpf = filterCmds(rx);

                const float drive_torque = driveControl(rx_lpf.drive);
                const float steer_torque = steerControl(rx_lpf.steer);

                right_motor.set_input_torque_msg.Input_Torque = (drive_torque + steer_torque) * 0.5f;
                left_motor.set_input_torque_msg.Input_Torque  = (drive_torque - steer_torque) * 0.5f;

                if (fabsf(imu.pitch) > 20.0f) {
                    vertical_timer.reset();
                    next_state = State::Idle;
                    can_sendMsg(left_motor.encode(ODriveCAN::kSetAxisStateMsg));
                    can_sendMsg(right_motor.encode(ODriveCAN::kSetAxisStateMsg));
                }
            } break;

            case State::Error:
            default: {
                left_motor.set_axis_state_msg.Axis_Requested_State  = AXIS_STATE_IDLE;
                right_motor.set_axis_state_msg.Axis_Requested_State = AXIS_STATE_IDLE;

                can_sendMsg(left_motor.encode(ODriveCAN::kSetAxisStateMsg));
                can_sendMsg(right_motor.encode(ODriveCAN::kSetAxisStateMsg));

                left_motor.set_input_torque_msg.Input_Torque  = 0.0f;
                right_motor.set_input_torque_msg.Input_Torque = 0.0f;

            } break;
        }

        sendCan();

        uint32_t end = micros();

#ifdef DEBUG
        if ((size_t)state < StateStrs.size()) {
            Serial.println(StateStrs[(size_t)state]);
        }

        Serial.printf("States L: %s\n", ODriveStateStrings[left_motor.heartbeat_msg.Axis_State]);
        Serial.printf("States R: %s\n", ODriveStateStrings[right_motor.heartbeat_msg.Axis_State]);
        // Serial.printf("Quat: w: %4.2f, i: %4.2f, j: %4.2f, k: %4.2f\n", imu.qw, imu.qi, imu.qj, imu.qk);
        Serial.printf("Angles R: %4.2f P: %4.2f Y: %4.2f\n", imu.roll, imu.pitch, imu.yaw);
        // Serial.printf("Rates  R: %4.2f P: %4.2f Y: %4.2f\n", imu.roll_rate, imu.pitch_rate, imu.yaw_rate);
        Serial.printf("Speeds L: %4.2f R: %4.2f\n", left_motor.get_encoder_estimates_msg.Vel_Estimate, right_motor.get_encoder_estimates_msg.Vel_Estimate);
        Serial.printf("%dus %3.1f%%\n", end - start, (end - start) / 100.0f);
        Serial.println();
#endif
    }
}

float driveControl(const float vel_target) {
    // Pitch angle and rate from IMU

    // Motor speeds
    const float vel_right = right_motor.get_encoder_estimates_msg.Vel_Estimate * (kWheelDiameter * kPi);  // [m/s] right wheel speed
    const float vel_left  = left_motor.get_encoder_estimates_msg.Vel_Estimate * (kWheelDiameter * kPi);   // [m/s] left wheel speed

    // TODO:  Verify yaw rate calculation matches gyro reading
    const float vel_actual = (vel_right + vel_left) / 2.0f - imu.pitch_rate;  // [m/s] Vehicle speed
    const float yaw_rate   = (vel_right - vel_left) / (2.0f * kTrackWidth);   // [rad/s] Vehicle yaw Rate, per motor sensors
    const float gyro_yaw   = imu.yaw;                                         // [rad/s] Vehicle yaw rate, per gyro

    // Run P/PI/P control loop
    const float pitch_cmd      = vel_controller.update(true, vel_target, vel_actual, kControlLoopPeriod);
    const float pitch_rate_cmd = pitch_controller.update(true, pitch_cmd, imu.pitch, kControlLoopPeriod);

    // Pitch rate controller outputs an acceleration, and Tau = J * alpha
    const float torque_cmd = pitch_rate_controller.update(true, pitch_rate_cmd, imu.pitch_rate, kControlLoopPeriod) * kJ;

    return torque_cmd;
}

float steerControl(const float yaw_target) {
    return 0.0f;
}

void can_onReceive(int packetSize) {
    can_Message_t rxmsg;
    rxmsg.id = CAN.packetId();
    CAN.readBytes(rxmsg.data, 8);

    switch (rxmsg.id >> ODriveCAN::kNumCmdIdBits) {
        case 0: left_motor.decode(rxmsg); break;
        case 1: right_motor.decode(rxmsg); break;
        default: break;
    }
}

void can_sendMsg(const can_Message_t& msg) {
    CAN.beginPacket(msg.id);
    CAN.write(msg.data, msg.dlc);
    CAN.endPacket();
}

void sendCan() {
    can_sendMsg(left_motor.encode(ODriveCAN::kSetInputTorqueMsg));
    can_sendMsg(right_motor.encode(ODriveCAN::kSetInputTorqueMsg));
}

void blink(const uint32_t blink_period_ms) {
    static uint32_t last_blink = millis();
    static bool led_state      = false;

    const uint32_t now = millis();
    if ((now - last_blink) >= (blink_period_ms / 2UL)) {
        last_blink = now;
        led_state  = !led_state;

        digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
    }
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