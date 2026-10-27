#include "drivetrain.hpp"
#include "robot_config.hpp"

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

// Starting PID values only: tune before using autonomous motions.
lemlib::ControllerSettings linearController(
    10,  // kP
    0,   // kI
    3,   // kD
    3,   // integral anti-windup range (inches)
    1,   // small error range (inches)
    100, // small error timeout (ms)
    3,   // large error range (inches)
    500, // large error timeout (ms)
    20   // slew
);
lemlib::ControllerSettings angularController(
    2,   // kP
    0,   // kI
    10,  // kD
    3,   // integral anti-windup range (degrees)
    1,   // small error range (degrees)
    100, // small error timeout (ms)
    3,   // large error range (degrees)
    500, // large error timeout (ms)
    0    // slew disabled
);

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

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors,
                       &throttleCurve, &steerCurve);
