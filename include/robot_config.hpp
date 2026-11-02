#pragma once

#include <cstdint>
#include <initializer_list>
#include "lemlib/api.hpp"

// Existing template values: verify ports, directions, and measurements on your robot.
namespace config {
// Negative motor ports reverse that motor. Positive power must drive each side forward.
inline constexpr std::initializer_list<std::int8_t> leftMotorPorts = {1, -2};
inline constexpr std::initializer_list<std::int8_t> rightMotorPorts = {9, -10};
inline constexpr auto driveGearset = pros::MotorGearset::blue; // 600 RPM cartridges
inline constexpr std::uint8_t imuPort = 3;
inline constexpr bool imuInstalled = true;
// Direction test: changed from -8 to +8 so the odometry pod reports the opposite sign.
// Verify a straight forward push increases Y without changing heading.
inline constexpr std::int8_t verticalEncoderPort = 8; // positive uses the sensor's normal direction

inline constexpr float trackWidth = 10; // inches, between left/right wheel centers
inline constexpr float driveWheelDiameter = 2.75;
inline constexpr float driveWheelRpm = 450; // wheel RPM AFTER external gearing
inline constexpr float horizontalDrift = 0; // starting value for omni wheels; tune on robot
inline constexpr float trackingWheelDiameter = 2.0;
inline constexpr float verticalWheelOffset = 0; // inches; right of tracking center is positive

inline constexpr float joystickDeadband = 3; // out of 127
inline constexpr float minimumDriveOutput = 10; // out of 127; tune for your drivetrain
inline constexpr float driveCurveGain = 1.019;

static_assert(trackWidth > 0 && driveWheelDiameter > 0 && driveWheelRpm > 0);
static_assert(trackingWheelDiameter > 0);
} // namespace config
