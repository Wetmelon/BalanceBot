
#include <Arduino_CAN.h>
#include <Wire.h>

#include <algorithm>

#include "PIController.hpp"
#include "SparkFun_BNO080_Arduino_Library.h"
#include "can_simple/can_simple_messages.hpp"

// Objects
BNO080 imu;
ODriveCAN left_motor{0};   // Node ID 0
ODriveCAN right_motor{1};  // Node ID 1

// Constants
constexpr float kControlLoopPeriod = 0.01f;  // sec
constexpr float kRadToDeg          = 360.0 / TWO_PI;
constexpr float kPi                = PI;

constexpr float kWheelDiameter = 0.1524f;  // [m] Wheel Diameter
constexpr float kTrackWidth    = 1.0f;     // [m] Distance between wheels

void readCAN() {
    if (CAN.available()) {
        const CanMsg rxmsg = CAN.read();
        switch (rxmsg.id >> ODriveCAN::kNumCmdIdBits) {
            case 0: left_motor.decode(rxmsg); break;
            case 1: right_motor.decode(rxmsg); break;
            default: break;
        }
    }
}

void sendCan() {
    const CanMsg left_torque_msg  = left_motor.encode(ODriveCAN::kSetInputTorqueMsg);
    const CanMsg right_torque_msg = right_motor.encode(ODriveCAN::kSetInputTorqueMsg);

    CAN.write(left_torque_msg);
    CAN.write(right_torque_msg);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        // Wait for serial port to become avaialble
    }

    Serial.println(F("Connected to serial at 115200 baud."));

    if (!CAN.begin(CanBitRate::BR_250k)) {
        Serial.println(F("CAN.begin(...) failed."));
        // Flash LED?
        for (;;) {
        }
    }

    Serial.println(F("Connected to CAN at 250kbps"));

    Wire.begin();  // Enable i2c
    imu.begin();

    Wire.setClock(400'000);  // Increase i2c data rate to 400khz

    // 100hz reporting
    imu.enableRotationVector(10);
    imu.enableGyro(10);
}

void loop() {
    static uint32_t last_run = micros();
    if ((micros() - last_run) > (uint32_t)(kControlLoopPeriod * 1e6f)) {
        last_run = micros();

        readCAN();

        if (imu.dataAvailable()) {
            const float imu_pitch  = imu.getPitch();  // [rad] Pitch angle
            const float pitch_rate = imu.getGyroX();  // [rad/s] Rate of rotation about pitch axis

            const float vel_right = right_motor.get_encoder_estimates_msg.Vel_Estimate * (kWheelDiameter * kPi);  // [m/s] right wheel speed
            const float vel_left  = left_motor.get_encoder_estimates_msg.Vel_Estimate * (kWheelDiameter * kPi);   // [m/s] left wheel speed

            // TODO:  Verify yaw rate calculation matches gyro reading
            const float vel_actual = (vel_right + vel_left) / 2.0f;                  // [m/s] Vehicle speed
            const float yaw_rate   = (vel_right - vel_left) / (2.0f * kTrackWidth);  // [rad/s] Vehicle Yaw Rate, per motor sensors
            const float gyro_yaw   = imu.getGyroZ();                                 // [rad/s] Vehicle yaw rate, per gyro

            const float vel_target   = 0.0f;  // Get velocity target from somewhere
            const float pitch_target = (vel_target - vel_actual);
        }

        sendCan();
    }
}
