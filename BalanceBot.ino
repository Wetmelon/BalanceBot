
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
constexpr float kControlLoopPeriod = 1.0f;  // sec
constexpr float kRadToDeg          = 360.0 / TWO_PI;
constexpr float kPi                = PI;

constexpr float kWheelDiameter = 0.1524f;  // [m] Wheel Diameter
constexpr float kTrackWidth    = 1.0f;     // [m] Distance between wheels
constexpr float kJ             = 1.0f;     // [Nm/(rad/s^2)] Rotational inertia

void readCAN(int packetSize) {
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

    // Adafruit Feather
    CAN.write(left_torque_msg.data, 8);
    CAN.write(right_torque_msg.data, 8);
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PIN_CAN_STANDBY, OUTPUT);
    pinMode(PIN_CAN_BOOSTEN, OUTPUT);

    // Enable CAN transceiver
    digitalWrite(PIN_CAN_STANDBY, false);  // turn off STANDBY
    digitalWrite(PIN_CAN_BOOSTEN, true);   // turn on booster

    Serial.begin(115200);
    Serial.println(F("Connected to serial at 115200 baud."));

    if (!CAN.begin(250000L)) {
        Serial.println(F("CAN.begin(...) failed."));
        // Flash LED?
        for (;;) {
            delay(10);
        }
    }

    CAN.onReceive(readCAN);

    Serial.println(F("Connected to CAN at 250kbps"));

    Wire.begin();  // Enable i2c
    imu.begin();

    Wire.setClock(400000);  // Increase i2c data rate to 400khz

    // // 100hz reporting
    // imu.enableRotationVector(10);
    // imu.enableGyro(10);
}

void loop() {
    static bool led_state    = LOW;
    static uint32_t last_run = micros();
    if ((micros() - last_run) > (uint32_t)(kControlLoopPeriod * 1e6f)) {
        last_run = micros();

        led_state = !led_state;
        digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
        Serial.print("vbus: ");
        Serial.println(left_motor.get_bus_voltage_current_msg.Bus_Voltage);

        const float vel_target = 0.0f;  // Get velocity target from somewhere
        const float yaw_target = 0.0f;  // Get yaw rate target from somewhere

        // Only bother to update the controller when we have new data available from the IMU

        // Pitch angle and rate from IMU
        const float pitch_angle = imu.getPitch();  // [rad] Pitch angle
        const float pitch_rate  = imu.getGyroX();  // [rad/s] Rate of rotation about pitch axis

        // Motor speeds
        const float vel_right = right_motor.get_encoder_estimates_msg.Vel_Estimate * (kWheelDiameter * kPi);  // [m/s] right wheel speed
        const float vel_left  = left_motor.get_encoder_estimates_msg.Vel_Estimate * (kWheelDiameter * kPi);   // [m/s] left wheel speed

        // TODO:  Verify yaw rate calculation matches gyro reading
        const float vel_actual = (vel_right + vel_left) / 2.0f;                  // [m/s] Vehicle speed
        const float yaw_rate   = (vel_right - vel_left) / (2.0f * kTrackWidth);  // [rad/s] Vehicle Yaw Rate, per motor sensors
        const float gyro_yaw   = imu.getGyroZ();                                 // [rad/s] Vehicle yaw rate, per gyro

        // Run P/PI/P control loop
        const float pitch_cmd      = vel_controller.update(true, vel_target, vel_actual, kControlLoopPeriod);
        const float pitch_rate_cmd = pitch_controller.update(true, pitch_cmd, pitch_angle, kControlLoopPeriod);

        // Pitch rate controller outputs an acceleration, and Tau = J * alpha
        const float torque_cmd = pitch_rate_controller.update(true, pitch_rate_cmd, pitch_rate, kControlLoopPeriod) * kJ;

        left_motor.set_input_torque_msg.Input_Torque  = torque_cmd * 0.5f;
        right_motor.set_input_torque_msg.Input_Torque = torque_cmd * 0.5f;

        sendCan();
    }
}
