#include "main.h"
#include "drivetrain.hpp"
#include "robot_config.hpp"

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, config::imuInstalled ? "Calibrating: keep robot still"
                                              : "Starting: IMU disabled");
    chassis.calibrate(config::imuInstalled);

    static pros::Task screenTask([] {
        while (true) {
            const auto pose = chassis.getPose();
            pros::lcd::print(0, "X: %.2f in", pose.x);
            pros::lcd::print(1, "Y: %.2f in", pose.y);
            pros::lcd::print(2, "Heading: %.2f deg", pose.theta);
            pros::delay(100);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 3000, {}, false); // 24 inches forward; wait for completion
}
