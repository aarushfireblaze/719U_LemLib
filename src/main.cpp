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
            pros::delay(50); // Space controller screen commands apart.
            controller.print(1, 0, "Angle:%7.2f deg  ", pose.theta);
            pros::lcd::print(0, "X: %.2f in", pose.x);
            pros::lcd::print(1, "Y: %.2f in", pose.y);
            pros::lcd::print(2, "Heading: %.2f deg", pose.theta);
            // Lift telemetry disabled while testing chassis turning.
            // pros::lcd::print(3, "Lift: %.1f deg", lift.LiftPos.load());
            // pros::lcd::print(4, "Lift enc: %.1f / %.1f", lift.Motor20Pos.load(), lift.Motor5Pos.load());
            // pros::lcd::print(5, "Lift: %s", lift.Faulted.load() ? "FAULT - stopped" :
            //                  lift.EncoderMismatch.load() ? "encoder difference" : "ready");
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
    // Curved approach to (24, 24), ending facing +X (90 degrees).
    // Use a trial cornering value because the chassis horizontalDrift is still zero.
    chassis.moveToPose(24, 24, 90, 10000,
                       {.horizontalDrift = 8, .lead = 0.6, .maxSpeed = 127}, false);

    // Chain into the turn within 3 inches; higher minimum speed reduces approach slowdown.
    // chassis.moveToPoint(0, 24, 10000,
    //                     {.maxSpeed = 127, .minSpeed = 40, .earlyExitRange = 3}, false);
    // Chain into the final drive within 3 degrees of facing +X.
    // chassis.turnToHeading(90, 10000,
    //                       {.maxSpeed = 127, .minSpeed = 20, .earlyExitRange = 3}, false);
    // Forward another 48 inches along the new heading, then settle normally.
    // chassis.moveToPoint(48, 24, 10000, {.maxSpeed = 127}, false);
    // Standalone 180-degree turn test with normal settling.
    // chassis.turnToHeading(180, 10000, {.maxSpeed = 80}, false);
    pros::lcd::print(7, "Motion ended: %lu ms",
                     static_cast<unsigned long>(pros::millis() - started));
}
