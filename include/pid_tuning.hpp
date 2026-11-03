#pragma once

#include "lemlib/api.hpp"

namespace tuning {
// Edit values here, rebuild, and upload. These are starting values, not tuned gains.
// For initial kP/kD tuning, zero anti-windup, error ranges/timeouts, and slew (see README).
inline const lemlib::ControllerSettings linearController(
    5.35,   // kP: straight-line correction
    0,   // kI: leave zero while tuning kP/kD
    3,   // kD: damping for the 48-inch test
    3,   // integral anti-windup range (inches)
    0,   // small error range disabled for lateral PID tuning
    0,   // small error timeout disabled during tuning
    0,   // large error range disabled for lateral PID tuning
    0,   // large error timeout disabled during tuning
    0    // slew disabled for linear movement tuning
);
inline const lemlib::ControllerSettings angularController(
    2,   // kP: heading correction
    0,   // kI: leave zero while tuning kP/kD
    10,  // kD: turn damping
    3,   // integral anti-windup range (degrees)
    1,   // small error range (degrees)
    100, // small error timeout (ms)
    3,   // large error range (degrees)
    500, // large error timeout (ms)
    0    // slew disabled
);

} // namespace tuning
