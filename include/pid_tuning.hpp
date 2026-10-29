#pragma once

#include "lemlib/api.hpp"

namespace tuning {
// Edit values here, rebuild, and upload. These are starting values, not tuned gains.
// For initial kP/kD tuning, zero anti-windup, error ranges/timeouts, and slew (see README).
inline const lemlib::ControllerSettings linearController(
    10,  // kP: straight-line correction
    0,   // kI: leave zero while tuning kP/kD
    3,   // kD: straight-line damping
    3,   // integral anti-windup range (inches)
    1,   // small error range (inches)
    100, // small error timeout (ms)
    3,   // large error range (inches)
    500, // large error timeout (ms)
    20   // slew; 0 disables acceleration limiting
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
