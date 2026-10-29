#include "drivetrain.hpp"
#include "robot_config.hpp"
#include "pid_tuning.hpp"

// Keep construction in one file: the chassis depends on these objects already existing.
namespace {
pros::MotorGroup leftMotors(config::leftMotorPorts, config::driveGearset);
pros::MotorGroup rightMotors(config::rightMotorPorts, config::driveGearset);
pros::Imu imu(config::imuPort);
pros::Rotation verticalEncoder(config::verticalEncoderPort);
lemlib::TrackingWheel verticalWheel(&verticalEncoder, config::trackingWheelDiameter,
                                   config::verticalWheelOffset);

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, config::trackWidth,
                             config::driveWheelDiameter, config::driveWheelRpm,
                             config::horizontalDrift);

lemlib::OdomSensors sensors(
    &verticalWheel,
    nullptr, // no second vertical tracking wheel; LemLib uses drive motor encoders
    nullptr, // no horizontal tracking wheels installed
    nullptr,
    &imu
);

lemlib::ExpoDriveCurve throttleCurve(config::joystickDeadband, config::minimumDriveOutput,
                                    config::driveCurveGain);
lemlib::ExpoDriveCurve steerCurve(config::joystickDeadband, config::minimumDriveOutput,
                                 config::driveCurveGain);
} // namespace

lemlib::Chassis chassis(drivetrain, tuning::linearController, tuning::angularController, sensors,
                       &throttleCurve, &steerCurve);
