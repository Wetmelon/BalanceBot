#pragma once

#include <algorithm>

#include "./SparkFun_BNO080_Arduino_Library.h"

namespace odrv {
template <class T>
constexpr const T& clamp(const T& x, const T& lo, const T& hi) {
    return std::min(std::max(x, lo), hi);
}

struct lpf {
    lpf(float Ts, float Tau) : _alpha(Ts / (Ts + Tau)) {}

    float step(float signal) {
        _state += _alpha * (signal - _state);
        return _state;
    }

    void reset() {
        _state = 0.0f;
    }

   private:
    float _state       = 0.0f;
    const float _alpha = 0.0f;
};

struct PIControllerClass {
    struct Settings_t {
        float iterm_min  = -0.1f;
        float iterm_max  = 0.1f;
        float output_min = 1.0f;
        float output_max = 1.0f;

        float Kp = 1.0f;
        float Ki = 0.0f;
        float Kd = 0.0f;
    };

    Settings_t settings;

    float update(bool enable, float target, float actual, float Ts) {
        const float error  = target - actual;
        const float p_term = error * settings.Kp;
        const float d_term = actual - last_actual;  // Derivative on measurement

        i_term += error * settings.Kp * Ts;
        i_term = odrv::clamp(i_term, settings.iterm_min, settings.iterm_max);
        i_term = enable ? i_term : 0.0f;

        return odrv::clamp(p_term + i_term + d_term, settings.output_min, settings.output_max);
    }

   private:
    float i_term      = 0;
    float last_actual = 0;
};

struct Timer {
    Timer(uint32_t timeout_ms) : _timeout(timeout_ms) {}

    void setTimeout(uint32_t timeout_ms) {
        _timeout = timeout_ms;
    }

    void reset() {
        _last_time = millis();
    }

    bool isExpired(bool reset_on_expired = false) {
        const uint32_t now = millis();
        if ((now - _last_time) >= _timeout) {
            _last_time = now - _timeout;  // Clamp _last_time so it doesn't break in rollover
            return true;
        }
        return false;
    }

   private:
    uint32_t _timeout   = 0;
    uint32_t _last_time = 0;
};

struct ImuWrapper {
    float pitch      = 0.0f;
    float pitch_rate = 0.0f;
    float roll       = 0.0f;
    float roll_rate  = 0.0f;
    float yaw        = 0.0f;
    float yaw_rate   = 0.0f;

    void begin() {
        // Request the IMU send the rotation vector and IMU at 100Hz
        imu.begin();
        imu.enableGameRotationVector(10);  // Gyro and Accel only
        // imu.enableGyro(10);
    }

    void step() {
        if (imu.dataAvailable()) {
            roll  = imu.getRoll();
            pitch = imu.getPitch();
            yaw   = imu.getYaw();

            roll_rate  = imu.getGyroX();
            pitch_rate = imu.getGyroY();
            yaw_rate   = imu.getGyroZ();
        }
    }

   private:
    BNO080 imu{};
    bool _data_valid = false;
};

}  // namespace odrv