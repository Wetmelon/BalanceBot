
// Uno R4
// #include <Arduino_CAN.h>

// Adafruit Feather M4 CAN
#include <Adafruit_NeoPixel.h>
#include <CANSAME5x.h>

#include <array>

#include "ODriveEnums.h"
#include "can_simple/can_helpers.hpp"
#include "can_simple/can_simple_messages.hpp"
#include "utils.hpp"

#define DEBUG

// Objects
Adafruit_NeoPixel pixel{1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ400};

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
    odrv::blink(1000);

    // Read IMU
    imu.read();

    // We run the control loop when there's data available from the IMU (100Hz)
    if (loop_timer.isExpired()) {
        // Serial.printf("Loop\n");
        loop_timer.reset();

        state = next_state;

        switch (state) {
            case State::Idle: {
                left_motor.set_axis_state_msg.Axis_Requested_State  = AXIS_STATE_IDLE;
                right_motor.set_axis_state_msg.Axis_Requested_State = AXIS_STATE_IDLE;

                left_motor.set_input_torque_msg.Input_Torque  = 0.0f;
                right_motor.set_input_torque_msg.Input_Torque = 0.0f;

                // If pitch is within 5 degrees for 2 seconds, enable motors
                if (fabsf(imu.pitch) > 5.0f) {
                    vertical_timer.reset();
                } else if (vertical_timer.isExpired() && (!imu.getIsTimedOut())) {
                    next_state = State::Active;

                    left_motor.set_axis_state_msg.Axis_Requested_State  = AXIS_STATE_CLOSED_LOOP_CONTROL;
                    right_motor.set_axis_state_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;

                    can_sendMsg(left_motor.encode(ODriveCAN::kSetAxisStateMsg));
                    can_sendMsg(right_motor.encode(ODriveCAN::kSetAxisStateMsg));

                    left_motor.set_controller_mode_msg.Control_Mode  = CONTROL_MODE_TORQUE_CONTROL;
                    right_motor.set_controller_mode_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
                    left_motor.set_controller_mode_msg.Input_Mode    = INPUT_MODE_PASSTHROUGH;
                    right_motor.set_controller_mode_msg.Input_Mode   = INPUT_MODE_PASSTHROUGH;

                    can_sendMsg(left_motor.encode(ODriveCAN::kSetControllerModeMsg));
                    can_sendMsg(right_motor.encode(ODriveCAN::kSetControllerModeMsg));

                    left_motor.set_limits_msg.Velocity_Limit  = 1000.0f;
                    right_motor.set_limits_msg.Velocity_Limit = 1000.0f;
                    left_motor.set_limits_msg.Current_Limit   = 40.0f;
                    right_motor.set_limits_msg.Current_Limit  = 40.0f;

                    can_sendMsg(left_motor.encode(ODriveCAN::kSetLimitsMsg));
                    can_sendMsg(right_motor.encode(ODriveCAN::kSetLimitsMsg));
                }

            } break;

            case State::Active: {
                next_state = State::Active;

                // TODO:  Reset rx_lpf on disable
                const CmdPair rx     = getControllerCmds();
                const CmdPair rx_lpf = filterCmds(rx);

                // Check for errors
                bool pitch_over  = fabsf(imu.pitch) > 20.0f;
                bool imu_timeout = imu.getIsTimedOut();

                bool left_error  = left_motor.heartbeat_msg.Axis_Error != 0;
                bool right_error = right_motor.heartbeat_msg.Axis_Error != 0;

                if (pitch_over || imu_timeout || left_error || right_error) {
                    vertical_timer.reset();
                    next_state = State::Idle;

                    left_motor.set_axis_state_msg.Axis_Requested_State  = AXIS_STATE_IDLE;
                    right_motor.set_axis_state_msg.Axis_Requested_State = AXIS_STATE_IDLE;

                    can_sendMsg(left_motor.encode(ODriveCAN::kSetAxisStateMsg));
                    can_sendMsg(right_motor.encode(ODriveCAN::kSetAxisStateMsg));

                    right_motor.set_input_torque_msg.Input_Torque = 0.0f;
                    left_motor.set_input_torque_msg.Input_Torque  = 0.0f;
                } else {
                    const float drive_torque = driveControl(rx_lpf.drive);
                    const float steer_torque = steerControl(rx_lpf.steer);

                    right_motor.set_input_torque_msg.Input_Torque = +0.5f * (drive_torque + steer_torque);
                    left_motor.set_input_torque_msg.Input_Torque  = -0.5f * (drive_torque - steer_torque);
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

float driveControl(const float vel_target) {
    // Pitch angle and rate from IMU

    // Motor speeds
    const float vel_right = right_motor.get_encoder_estimates_msg.Vel_Estimate * (kWheelDiameter * kPi);         // [m/s] right wheel speed
    const float vel_left  = -1.0f * left_motor.get_encoder_estimates_msg.Vel_Estimate * (kWheelDiameter * kPi);  // [m/s] left wheel speed

    // TODO:  Verify yaw rate calculation matches gyro reading
    const float vel_actual = (vel_right + vel_left) / 2.0f;  //  - imu.pitch_rate;  // [m/s] Vehicle speed
    // const float yaw_rate   = (vel_right - vel_left) / (2.0f * kTrackWidth);  // [rad/s] Vehicle yaw Rate, per motor sensors
    // const float gyro_yaw   = imu.yaw;                                        // [rad/s] Vehicle yaw rate, per gyro

    // Run P/PI/P control loop
    vel_controller.settings.Kp = 10.0f;
    vel_controller.settings.Ki = 1.0f;

    vel_controller.settings.iterm_min  = -10.0f;  // [deg]
    vel_controller.settings.output_min = -10.0f;  // [deg]

    vel_controller.settings.iterm_max  = 10.0f;  // [deg]
    vel_controller.settings.output_max = 10.0f;  // [deg]

    const float pitch_cmd = vel_controller.update(true, 0.0f, vel_actual, kControlLoopPeriod);

    pitch_controller.settings.Kp = 10.0f;
    pitch_controller.settings.Ki = 10.0f;

    pitch_controller.settings.iterm_min  = -100.0f;  // [deg/s]
    pitch_controller.settings.iterm_max  = 100.0f;   // [deg/s]
    pitch_controller.settings.output_min = -100.0f;  // [deg/s]
    pitch_controller.settings.output_max = 100.0f;   // [deg/s]

    const float pitch_rate_cmd = pitch_controller.update(true, pitch_cmd, imu.pitch, kControlLoopPeriod);

    // Pitch rate controller outputs an acceleration, and Tau = J * alpha
    pitch_rate_controller.settings.Kp = 0.1f;
    pitch_rate_controller.settings.Ki = 0.0f;

    pitch_rate_controller.settings.output_min = -10.0f;  // [Nm]
    pitch_rate_controller.settings.output_max = 10.0f;   // [Nm]

    const float torque_cmd = -1.0f * pitch_rate_controller.update(true, pitch_rate_cmd, imu.pitch_rate, kControlLoopPeriod) * kJ;

#ifdef DEBUG

    // Serial.printf("V: % 06.2f Pcmd: % 06.2f P: % 06.2f Rcmd: % 06.2f R: % 06.2f T: % 06.2f\n",
    //               vel_actual, pitch_cmd, imu.pitch, pitch_rate_cmd, imu.pitch_rate, torque_cmd);
    Serial.printf("% 06.2f,% 06.2f,% 06.2f,% 06.2f,% 06.2f,% 06.2f\n",
                  vel_actual, pitch_cmd, imu.pitch, pitch_rate_cmd, imu.pitch_rate, torque_cmd);

#endif

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
