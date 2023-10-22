#pragma once

#include <algorithm>

namespace odrv {

template <class T>
constexpr const T& clamp(const T& x, const T& lo, const T& hi) {
    return std::min(std::max(x, lo), hi);
}

struct PIDController {
    struct Settings_t {
        float iterm_min  = -1.0f;
        float iterm_max  = 1.0f;
        float output_min = -1.0f;
        float output_max = 1.0f;

        float Kp = 1.0f;
        float Ki = 0.0f;
        float Kd = 0.0f;
    };

    Settings_t settings;

    float update(bool enable, float target, float actual, float Ts) {
        const float error  = target - actual;
        const float p_term = error * settings.Kp;
        const float d_term = (actual - last_actual) * settings.Kd;  // Derivative on measurement

        i_term += error * settings.Ki * Ts;
        i_term = odrv::clamp(i_term, settings.iterm_min, settings.iterm_max);
        i_term = enable ? i_term : 0.0f;

        return odrv::clamp(p_term + i_term + d_term, settings.output_min, settings.output_max);
    }

   private:
    float i_term      = 0.0f;
    float last_actual = 0.0f;
};

}  // namespace odrv
