#pragma once

#include <cstdint>
#include <initializer_list>
#include "lemlib/api.hpp"

// Existing template values: verify ports, directions, and measurements on your robot.
namespace config {
// Negative motor ports reverse that motor. Positive power must drive each side forward.
inline constexpr std::initializer_list<std::int8_t> leftMotorPorts = {-5, 4, -3};
inline constexpr std::initializer_list<std::int8_t> rightMotorPorts = {6, -9, 7};
inline constexpr auto driveGearset = pros::MotorGearset::blue; // 600 RPM cartridges
inline constexpr std::uint8_t imuPort = 10;
inline constexpr std::int8_t verticalEncoderPort = -11; // negative reverses the sensor

inline constexpr float trackWidth = 10; // inches, between left/right wheel centers
inline constexpr float driveWheelDiameter = lemlib::Omniwheel::NEW_4;
inline constexpr float driveWheelRpm = 360; // wheel RPM AFTER external gearing
inline constexpr float horizontalDrift = 2; // starting value for omni wheels; tune on robot
inline constexpr float trackingWheelDiameter = lemlib::Omniwheel::NEW_275;
inline constexpr float verticalWheelOffset = 0; // inches; right of tracking center is positive

inline constexpr float joystickDeadband = 3; // out of 127
inline constexpr float minimumDriveOutput = 10; // out of 127; tune for your drivetrain
inline constexpr float driveCurveGain = 1.019;

static_assert(trackWidth > 0 && driveWheelDiameter > 0 && driveWheelRpm > 0);
static_assert(trackingWheelDiameter > 0);
} // namespace config
