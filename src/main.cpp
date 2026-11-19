#include "main.h"
#include "drivetrain.hpp"
#include "robot_config.hpp"
#include "lift.hpp"
#include "claw.hpp"

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
    const auto started = pros::millis();
    // Field coordinates in inches, field center = (0, 0), same as path.jerryio.
    chassis.setPose(-56.2, 0, 90);
    chassis.moveToPoint(-61.2, 0, 1000, {.forwards = false, .minSpeed = 100, .earlyExitRange = 0.5}, false);
    chassis.moveToPoint(-46.73, -0.05, 2000, {}, false);

    // Score the preload in the upper red Goal. Lift up halfway through the 14 in drive.
    chassis.turnToPoint(-46.73, -14.05, 1000, {.minSpeed = 20, .earlyExitRange = 5}, false);
    chassis.moveToPoint(-46.73, -14.05, 2000);
    chassis.waitUntil(7);
    lift.LiftUp(); // Keeps rising while the robot drives in.
    chassis.waitUntilDone();
    lift.LiftStop();
    // On the Goal: lower the lift, wait half a second, release before moving away.
    lift.LiftDown();
    pros::delay(600); // TUNE: how long the lift lowers.
    lift.LiftStop();
    pros::delay(500);
    openClaw();

    // Back up, then go straight to the middle Pin at (-23.55, -23.55), slow for the last 10 in.
    chassis.moveToPoint(-46.73, -0.25, 2000, {.forwards = false}, false);
    chassis.turnToPoint(-23.75, -23.19, 1000, {}, false);
    chassis.moveToPoint(-30.83, -16.13, 2000, {.minSpeed = 30, .earlyExitRange = 1}, false);
    chassis.moveToPoint(-23.75, -23.19, 2000, {.maxSpeed = 30}, false);
    closeClaw(); // Grab the Pin.
    pros::delay(200); // Let the piston grip before driving.

    // Score it in the upper red Goal. Full lift halfway through the 14.5 in drive.
    chassis.turnToPoint(-38.24, -23.54, 1000, {.minSpeed = 20, .earlyExitRange = 5}, false);
    chassis.moveToPoint(-38.24, -23.54, 2000);
    chassis.waitUntil(7);
    lift.LiftUp(); // Keeps rising while the robot drives in.
    chassis.waitUntilDone();
    lift.LiftStop();
    // On the Goal: lower the lift, wait half a second, release before moving away.
    lift.LiftDown();
    pros::delay(600); // TUNE: how long the lift lowers.
    lift.LiftStop();
    pros::delay(500);
    openClaw();

    // Back up to x = -23.75, face 225, go to the corner Pin at (-46, -46), slow for the last 10 in.
    chassis.moveToPoint(-23.75, -23.54, 2000, {.forwards = false}, false);
    chassis.turnToHeading(225, 1000, {}, false);
    chassis.moveToPoint(-38.96, -38.9, 2000, {.minSpeed = 30, .earlyExitRange = 1}, false);
    chassis.moveToPoint(-46, -46, 2000, {.maxSpeed = 30}, false);
    closeClaw(); // Grab the Pin.
    pros::delay(200); // Let the piston grip before driving.

    // Face 0 and score in the upper red Goal from below, stopping 9 in short. Full lift halfway (13.5 in drive).
    chassis.turnToHeading(0, 1000, {.minSpeed = 20, .earlyExitRange = 5}, false);
    chassis.moveToPoint(-46, -32.5, 2000);
    chassis.waitUntil(7);
    lift.LiftUp(); // Keeps rising while the robot drives in.
    chassis.waitUntilDone();
    lift.LiftStop();
    // On the Goal: lower the lift, wait half a second, release before moving away.
    lift.LiftDown();
    pros::delay(600); // TUNE: how long the lift lowers.
    lift.LiftStop();
    pros::delay(500);
    openClaw();

    pros::lcd::print(7, "Auton ended: %lu ms",
                     static_cast<unsigned long>(pros::millis() - started));
}

