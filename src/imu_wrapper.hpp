
#pragma once

#include <Wire.h>

#include "./SparkFun_BNO080_Arduino_Library.h"
#include "MkrRgb.hpp"
#include "utils.hpp"

constexpr float r2d(const float rad) {
    return rad * (180.0f / bot::kPi);
}

struct Quaternion {
    float w, x, y, z;

    Quaternion normalize() const {
        const float norm = std::sqrt(sq(w) + sq(x) + sq(y) + sq(z));
        return {w / norm, x / norm, y / norm, z / norm};
    }
};

struct Euler {
    float roll, pitch, yaw;

    Euler(const Quaternion& q) {
        const float xsqr = sq(q.x);
        const float ysqr = sq(q.y);
        const float zsqr = sq(q.z);

        // roll (x-axis rotation)
        const float t0 = +2.0f * (q.w * q.x + q.y * q.z);
        const float t1 = +1.0f - 2.0f * (xsqr + ysqr);
        roll           = std::atan2(t0, t1);

        // pitch (y-axis rotation)
        const float t2 = bot::clamp(+2.0f * (q.w * q.y - q.z * q.x), -1.0f, 1.0f);
        pitch          = std::asin(t2);

        // yaw (z-axis rotation)
        const float t3 = +2.0f * (q.w * q.z + q.x * q.y);
        const float t4 = +1.0f - 2.0f * (ysqr + zsqr);
        yaw            = std::atan2(t3, t4);
    }
};

struct ImuWrapper {
    float pitch      = 0.0f;
    float pitch_rate = 0.0f;
    float roll       = 0.0f;
    float roll_rate  = 0.0f;
    float yaw        = 0.0f;
    float yaw_rate   = 0.0f;

    bool begin() {
        Wire.begin();
        // Serial.println("Wire started");

        // _imu.enableDebugging(Serial);

        while (!_imu.begin(0x4A)) {
            pixel.setColor(127, 0, 0);
        }
        pixel.setColor(0, 0, 127);

        Wire.setClock(400000);
        // Serial.println("Wire clock set");

        // _imu.enableGyroIntegratedRotationVector(10);

        _imu.enableARVRStabilizedGameRotationVector(10);  // Gyro and Accel only
        _imu.enableUncalibratedGyro(10);                  // Gyro without the zero-clamping

        // Serial.println("IMU Configured");
        return true;
    }

    void read() {
        _data_valid = false;
        if (_imu.dataAvailable()) {
            // Serial.println('i');
            timer.reset();
            _data_valid = true;

            const Euler e{Quaternion{
                _imu.getQuatReal(),
                _imu.getQuatI(),
                _imu.getQuatJ(),
                _imu.getQuatK(),
            }};

            roll  = r2d(e.pitch);
            pitch = r2d(e.roll) - 90.0f;
            yaw   = r2d(e.yaw);

            roll_rate  = r2d(_imu.getUncalibratedGyroY());
            pitch_rate = r2d(_imu.getUncalibratedGyroX());
            yaw_rate   = r2d(_imu.getUncalibratedGyroZ());
        }

        _timeout = timer.isExpired();
    }

    bool getIsTimedOut() {
        return _timeout;
    }

    bool getIsDataValid() {
        return _data_valid;
    }

    BNO080 _imu{};

   private:
    bot::Timer timer{1000};
    bool       _data_valid = false;
    bool       _timeout    = false;
};

extern ImuWrapper imu;
