#pragma once

#include <algorithm>

namespace odrv {
template <class T>
constexpr const T& clamp(const T& x, const T& lo, const T& hi) {
    return std::min(std::max(x, lo), hi);
}
}  // namespace odrv