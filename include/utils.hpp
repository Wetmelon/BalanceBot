#pragma once

namespace bot {

// Numbers
constexpr float kPi                = PI;
constexpr float kControlLoopPeriod = 0.01f;  // [sec] Control loop period in sec

template <class T>
constexpr const T& clamp(const T& x, const T& lo, const T& hi) {
    return std::min(std::max(x, lo), hi);
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
