#pragma once

#include "lemlib/api.hpp"

extern lemlib::Chassis chassis;

// Blocks until the IMU has finished calibrating (true right away if no IMU is installed).
// Returns false if it timed out.
bool waitForImuCalibration();

// IMU total rotation in degrees, unwrapped (keeps counting past 360), clockwise positive.
// Raw = straight from the IMU; scaled = raw * IMU_SCALE (what LemLib uses). NaN if no IMU.
double imuRawRotation();
double imuScaledRotation();
