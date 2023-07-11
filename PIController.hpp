#pragma once

#include <algorithm>

#include "utils.hpp"

struct PIControllerClass {
    void update(bool enable, float target, float actual, float Ts) {
        float error  = target - actual;
        float p_term = error * Kp;
        float d_term = actual - last_actual;  // Derivative on measurement
        i_term += error * Kp * Ts;
        i_term = odrv::clamp(i_term, iterm_min, iterm_max);
        i_term = enable ? i_term : 0.0f;
    }

    float Kp = 1.0f;
    float Ki = 1.0f;
    float Kd = 0.0f;

    const float iterm_max = -10.0f;
    const float iterm_min = -10.0f;

   private:
    float i_term      = 0;
    float last_actual = 0;
};
