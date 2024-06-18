#pragma once

namespace bot {

// Numbers
constexpr float kPi                = PI;
constexpr float kControlLoopPeriod = 0.012f;  // [sec] Control loop period in sec TODO HACK hardcoded, need to get from actual task loop rate setting

template <class T>
constexpr const T& clamp(const T& x, const T& lo, const T& hi) {
    return std::min(std::max(x, lo), hi);
}

float remap_joystick(float cmd) {
    float k2            = 0.5f;
    float k1            = 1.0f - k2;
    float deadband_each = 0.01f;  // total deadband width is 2*deadband_each
                                  // calc in absolute value, copy sign at the end

    // deadband
    float cmd_abs = fabsf(cmd);
    if (cmd_abs < deadband_each) {
        cmd_abs = 0.0f;
    } else {
        // remap from [deadband, 1] to [0, 1]
        cmd_abs = (cmd_abs - deadband_each) / (1.0f - deadband_each);
    }

    // quadratic map
    cmd_abs = (k1 * cmd_abs) + k2 * sq(cmd_abs);

    // restore sign
    cmd = copysignf(cmd_abs, cmd);

    return cmd;
}

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

struct LPF {
    LPF(float Ts, float Tau) : _alpha(Ts / (Ts + Tau)) {}

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

    bool isExpired() {
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

}  // namespace bot
