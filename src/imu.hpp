#pragma once

#include <FreeRTOS_SAMD21.h>
#include <Wire.h>

#include "MkrRgb.hpp"
#include "SparkFun_BNO080_Arduino_Library.h"
#include "balance_controller.hpp"
#include "damp/toolbox/timing.hpp"

/// BNO080 on I²C: game rotation vector (gyro + accel, no magnetometer) and raw gyro at 100 Hz.
struct ImuWrapper {
    struct Sample {
        float pitch      = 0.0f;   ///< [rad] bot lean, forward positive
        float pitch_rate = 0.0f;   ///< [rad/s]
        bool  fresh      = false;  ///< a report arrived within kTimeoutMs
    };

    void begin() {
        Wire.begin();
        while (!bno_.begin(0x4A)) {
            pixel.setColor(127, 0, 0);
        }
        pixel.setColor(0, 0, 127);
        Wire.setClock(400000);
        bno_.enableARVRStabilizedGameRotationVector(10);  // [ms]
        bno_.enableUncalibratedGyro(10);                  // [ms] no zero-rate clamping
    }

    /// Poll; call every 1 ms.
    void read() {
        if (bno_.dataAvailable()) {
            const damp::Quaternion<float> q{bno_.getQuatReal(), bno_.getQuatI(), bno_.getQuatJ(), bno_.getQuatK()};
            const float                   pitch = bot::pitch_from_imu(q);
            const float                   rate  = bno_.getUncalibratedGyroX();
            timeout_.reset();
            taskENTER_CRITICAL();
            sample_ = {.pitch = pitch, .pitch_rate = rate, .fresh = true};
            taskEXIT_CRITICAL();
        }
        if (timeout_.tick(1U)) {
            taskENTER_CRITICAL();
            sample_.fresh = false;
            taskEXIT_CRITICAL();
        }
    }

    /// The last reading, copied whole so pitch and rate come from the same report.
    Sample latest() {
        taskENTER_CRITICAL();
        const Sample s = sample_;
        taskEXIT_CRITICAL();
        return s;
    }

   private:
    static constexpr uint32_t kTimeoutMs = 1000;

    BNO080                  bno_{};
    damp::Timeout<uint32_t> timeout_{kTimeoutMs};
    Sample                  sample_{};
};

extern ImuWrapper imu;
