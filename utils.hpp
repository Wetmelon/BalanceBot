#pragma once

#include <WiFiNINA.h>
#include <Wire.h>
#include <utility/wifi_drv.h>

#include <algorithm>

#include "./SparkFun_BNO080_Arduino_Library.h"

constexpr float kPi = PI;

namespace odrv {
struct MKRrgb {
    void setup() {
        WiFiDrv::pinMode(GREEN, OUTPUT);
        WiFiDrv::pinMode(RED, OUTPUT);
        WiFiDrv::pinMode(BLUE, OUTPUT);
        setColor(0, 0, 0);
    }

    void setColor(uint8_t r, uint8_t g, uint8_t b) {
        WiFiDrv::analogWrite(GREEN, r);  // GREEN
        WiFiDrv::analogWrite(RED, g);    // RED
        WiFiDrv::analogWrite(BLUE, b);   // BLUE
    }

   private:
    enum {
        GREEN = 25,
        RED   = 26,
        BLUE  = 27,
    };
};

// On-board NeoPixel object
MKRrgb pixel;

void blink(const uint32_t blink_period_ms) {
    static uint32_t last_blink = millis();
    static bool     led_state  = false;

    const uint32_t now = millis();
    if ((now - last_blink) >= (blink_period_ms / 2UL)) {
        last_blink = now;
        led_state  = !led_state;

        digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
    }
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
    float       _state = 0.0f;
    const float _alpha = 0.0f;
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

    float qw, qi, qj, qk;

    void begin() {
        // Request the IMU send the rotation vector and IMU at 100Hz
        // Enable i2c at 400kHz
        Wire.begin();

        // Flush potentially stuck comms
        Wire.endTransmission();
        Wire.endTransmission();

        Serial.println("Wire started");

        // _imu.enableDebugging(Serial);

        if (!_imu.begin(0x4A)) {
            Serial.println("i2c startup failed!");
            pixel.setColor(127, 0, 0);

            while (true) {
                blink(2000);
                delay(1);
            }
        } else {
            Serial.println("IMU started");
            pixel.setColor(0, 127, 0);
        }

        Wire.setClock(400000);
        Serial.println("Wire clock set");

        _imu.enableGameRotationVector(10);  // Gyro and Accel only
        _imu.enableUncalibratedGyro(10);    // Gyro without the zero-clamping

        Serial.println("IMU Configured");
    }

    void read() {
        _data_valid = false;
        if (_imu.dataAvailable()) {
            timer.reset();
            _data_valid = true;

            // Sensor is oriented in such a way that imu roll is vehicle pitch
            pitch = _imu.getRoll() * (180.0f / kPi) - 90.0f;
            roll  = _imu.getPitch() * (180.0f / kPi);
            yaw   = _imu.getYaw() * (180.0f / kPi);

            pitch_rate = _imu.getUncalibratedGyroX() * (180.0f / kPi);
            roll_rate  = _imu.getUncalibratedGyroY() * (180.0f / kPi);
            yaw_rate   = _imu.getUncalibratedGyroZ() * (180.0f / kPi);
        }

        if (timer.isExpired())
            _timeout = true;
    }

    bool getIsTimedOut() {
        return _timeout;
    }

    bool getIsDataValid() {
        return _data_valid;
    }

   private:
    BNO080 _imu{};
    Timer  timer{1000};
    bool   _data_valid = false;
    bool   _timeout    = false;
};

}  // namespace odrv
