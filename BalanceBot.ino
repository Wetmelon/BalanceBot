
// Uno R4
// #include <Arduino_CAN.h>

// Adafruit Feather M4 CAN
#include <CANSAME5x.h>
#include <Wire.h>

#include "PIController.hpp"
#include "SparkFun_BNO080_Arduino_Library.h"
#include "can_simple/can_helpers.hpp"
#include "can_simple/can_simple_messages.hpp"

// Objects
BNO080 imu;
ODriveCAN left_motor{0};   // Node ID 0
ODriveCAN right_motor{1};  // Node ID 1

CANSAME5x CAN;

PIControllerClass vel_controller;
PIControllerClass pitch_controller;
PIControllerClass pitch_rate_controller;

// Constants
constexpr float kControlLoopPeriod = 0.01f;  // [sec]
constexpr float kRadToDeg          = 360.0 / TWO_PI;
constexpr float kPi                = PI;

constexpr float kWheelDiameter = 0.1524f;  // [m] Wheel Diameter
constexpr float kTrackWidth    = 1.0f;     // [m] Distance between wheels
constexpr float kJ             = 1.0f;     // [Nm/(rad/s^2)] Rotational inertia

struct CmdPair {
    float drive;
    float steer;
};

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_CAN_STANDBY, OUTPUT);
    pinMode(PIN_CAN_BOOSTEN, OUTPUT);

    // Enable CAN transceiver
    digitalWrite(PIN_CAN_STANDBY, false);  // turn off STANDBY
    digitalWrite(PIN_CAN_BOOSTEN, true);   // turn on booster

    Serial.begin(115200);
    Serial.println(F("Connected to serial at 115200 baud."));

    // Start CAN at 250kbps
    CAN.begin(250000);
    CAN.onReceive(onCanRx);

    Serial.println(F("Connected to CAN at 250kbps"));

    // Enable i2c at 400kHz
    Wire.begin();
    Wire.setClock(400000);

    // Request the IMU send the rotation vector and IMU at 100Hz
    imu.begin();
    imu.enableRotationVector((uint16_t)(kControlLoopPeriod * 1000.0f));
    imu.enableGyro((uint16_t)(kControlLoopPeriod * 1000.0f));
}

void loop() {
    static uint32_t last_run = millis();

    // Blink the LED at 1Hz
    blink(1000);

    // We run the control loop when there's data available from the IMU (100Hz)
    const uint32_t now = millis();
    if ((now - last_run) >= (uint32_t)(kControlLoopPeriod * 1000.0f)) {
        const CmdPair rx     = getControllerCmds();
        const CmdPair rx_lpf = filterCmds(rx);

        const float drive_torque = driveControl(rx_lpf.drive);
        const float steer_torque = steerControl(rx_lpf.steer);

        right_motor.set_input_torque_msg.Input_Torque = (drive_torque + steer_torque) * 0.5f;
        left_motor.set_input_torque_msg.Input_Torque  = (drive_torque - steer_torque) * 0.5f;

        sendCan();
    }
}

float driveControl(const float vel_target) {
    // Pitch angle and rate from IMU
    const float pitch_angle = imu.getPitch();  // [rad] Pitch angle in world frame
    const float pitch_rate  = imu.getGyroX();  // [rad/s] Rate of rotation about pitch axis in IMU frame

    // Motor speeds
    const float vel_right = right_motor.get_encoder_estimates_msg.Vel_Estimate * (kWheelDiameter * kPi);  // [m/s] right wheel speed
    const float vel_left  = left_motor.get_encoder_estimates_msg.Vel_Estimate * (kWheelDiameter * kPi);   // [m/s] left wheel speed

    // TODO:  Verify yaw rate calculation matches gyro reading
    const float vel_actual = (vel_right + vel_left) / 2.0f - pitch_rate;     // [m/s] Vehicle speed
    const float yaw_rate   = (vel_right - vel_left) / (2.0f * kTrackWidth);  // [rad/s] Vehicle yaw Rate, per motor sensors
    const float gyro_yaw   = imu.getGyroZ();                                 // [rad/s] Vehicle yaw rate, per gyro

    // Run P/PI/P control loop
    const float pitch_cmd      = vel_controller.update(true, vel_target, vel_actual, kControlLoopPeriod);
    const float pitch_rate_cmd = pitch_controller.update(true, pitch_cmd, pitch_angle, kControlLoopPeriod);

    // Pitch rate controller outputs an acceleration, and Tau = J * alpha
    const float torque_cmd = pitch_rate_controller.update(true, pitch_rate_cmd, pitch_rate, kControlLoopPeriod) * kJ;

    return torque_cmd;
}

float steerControl(const float yaw_target) {
    return 0.0f;
}

void onCanRx(int packetSize) {
    can_Message_t rxmsg;
    rxmsg.id = CAN.packetId();
    CAN.readBytes(rxmsg.data, 8);

    switch (rxmsg.id >> ODriveCAN::kNumCmdIdBits) {
        case 0: left_motor.decode(rxmsg); break;
        case 1: right_motor.decode(rxmsg); break;
        default: break;
    }
}

void sendCan() {
    const can_Message_t left_torque_msg  = left_motor.encode(ODriveCAN::kSetInputTorqueMsg);
    const can_Message_t right_torque_msg = right_motor.encode(ODriveCAN::kSetInputTorqueMsg);

    CAN.write(left_torque_msg.data, 8);
    CAN.write(right_torque_msg.data, 8);
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