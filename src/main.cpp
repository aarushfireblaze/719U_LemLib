#include "main.h"
#include "drivetrain.hpp"
#include "robot_config.hpp"
#include "lift.hpp"

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Starting lift...");
    lift.Initialize();
    pros::lcd::set_text(6, "Lift initialized; calibrating");
    pros::lcd::set_text(0, config::imuInstalled ? "Calibrating: keep robot still"
                                              : "Starting: IMU disabled");
    chassis.calibrate(config::imuInstalled);
    pros::lcd::set_text(6, "Ready; waiting for driver mode");

    static pros::Task screenTask([] {
        pros::Controller controller(pros::E_CONTROLLER_MASTER);
        controller.clear();
        pros::delay(100); // Space controller screen commands apart.
        while (true) {
            const auto pose = chassis.getPose();
            controller.print(0, 0, "X:%6.1f Y:%6.1f  ", pose.x, pose.y);
            pros::lcd::print(0, "X: %.2f in", pose.x);
            pros::lcd::print(1, "Y: %.2f in", pose.y);
            pros::lcd::print(2, "Heading: %.2f deg", pose.theta);
            pros::lcd::print(3, "Lift: %.1f deg", lift.LiftPos.load());
            pros::lcd::print(4, "Lift enc: %.1f / %.1f", lift.Motor20Pos.load(), lift.Motor5Pos.load());
            pros::lcd::print(5, "Lift: %s", lift.Faulted.load() ? "FAULT - stopped" :
                             lift.EncoderMismatch.load() ? "encoder difference" : "ready");
            pros::delay(100);
        }
    });
}

void disabled() {
    lift.LiftStop();
    pros::lcd::set_text(6, "Competition: DISABLED");
}

void competition_initialize() {}

void autonomous() {
    pros::lcd::set_text(6, "Mode: AUTONOMOUS");
    lift.LiftStop();
    chassis.setPose(0, 0, 0);
    const auto started = pros::millis();
    chassis.moveToPoint(0, 48, 10000, {.maxSpeed = 127}, false);
    pros::lcd::print(7, "Motion ended: %lu ms",
                     static_cast<unsigned long>(pros::millis() - started));
}
