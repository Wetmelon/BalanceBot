#pragma once

#include <algorithm>

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

   private:
    float _state       = 0.0f;
    const float _alpha = 0.0f;
};
}  // namespace odrv
