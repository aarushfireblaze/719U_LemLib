#pragma once

#include <cmath>

namespace lift_feedback {
constexpr double maximumMismatch = 15;

inline bool valid(double first, double second) {
    return std::isfinite(first) && std::isfinite(second);
}
inline bool mismatched(double first, double second) {
    return valid(first, second) && std::abs(first - second) > maximumMismatch;
}
inline double position(double first, double second) {
    return first / 2.0 + second / 2.0;
}
} // namespace lift_feedback
