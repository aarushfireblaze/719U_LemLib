#include "drivetrain.hpp"
#include "robot_config.hpp"
#include "pid_tuning.hpp"
#include "scaled_imu.hpp"
#include "localization/ekf_config.hpp"

// Keep construction in one file: the chassis depends on these objects already existing.
namespace {
pros::MotorGroup leftMotors(config::leftMotorPorts, config::driveGearset);
pros::MotorGroup rightMotors(config::rightMotorPorts, config::driveGearset);
ScaledImu imu(config::imuPort, ekf_config::IMU_SCALE); // LemLib sees raw rotation * IMU_SCALE
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
    config::imuInstalled ? &imu : nullptr
);

lemlib::ExpoDriveCurve throttleCurve(config::joystickDeadband, config::minimumDriveOutput,
                                    config::driveCurveGain);
lemlib::ExpoDriveCurve steerCurve(config::joystickDeadband, config::minimumDriveOutput,
                                 config::driveCurveGain);
} // namespace

bool waitForImuCalibration() { return !config::imuInstalled || imu.waitUntilCalibrated(); }
double imuRawRotation() { return config::imuInstalled ? imu.rawRotation() : NAN; }
double imuScaledRotation() { return config::imuInstalled ? imu.get_rotation() : NAN; }

lemlib::Chassis chassis(drivetrain, tuning::linearController, tuning::angularController, sensors,
                       &throttleCurve, &steerCurve);
