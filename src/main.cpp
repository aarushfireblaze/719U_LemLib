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
    // Start on the center line facing 270 (front toward the left wall), back straight up to x = -46.73.
    chassis.setPose(-60.2, 0, 270);
    chassis.moveToPoint(-46.978, 0, 2000, {.forwards = false}, false);

    // // Score the preload in the upper red Goal. Lift starts rising as the drive starts.
    chassis.turnToHeading(180, 1000, {}, false);
    lift.LiftUpFor(500); // Raises for 500 ms then holds on its own; returns instantly so the drive starts now.
    chassis.moveToPoint(-46.978, -17.64, 1000, {}, false);
    lift.LiftDown();
    pros::delay(600); // TUNE: how long the lift lowers.
    lift.LiftStop();
    openClaw();
    pros::delay(200);

    // // Back up to the center line.
    chassis.moveToPoint(-46.978, 0, 2000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 2}, false); // chained into the turn

    // Turn the toggle: face 90, back straight into the toggle on the left wall, then drive back to x = -46.73.
    chassis.turnToHeading(90, 1000, {.minSpeed = 20, .earlyExitRange = 2}, false); // chained into the back-up
    chassis.moveToPoint(-63, 0, 1500, {.forwards = false, .minSpeed = 100, .earlyExitRange = 0.5}, false); // fast: no slow-down into the toggle
    chassis.moveToPoint(-46.978, 0, 2000, {}, false);

    // Middle pin at (-23.75, -23.19): face it and stop 9 in short, on the line from (-46.978, 0).
    chassis.turnToPoint(-23.75, -23.75, 1000, {}, false);
    chassis.moveToPoint(-32.5, -15.73, 2000, {.maxSpeed = 40, .minSpeed = 30, .earlyExitRange = 1}, false);
    closeClaw(); // Grab the pin.
    lift.LiftUpFor(700); // Lift rises during the next move; returns instantly.
    // chassis.moveToPoint(-24, -24, 2000, {.maxSpeed = 62}, false);


    // Score it in the upper red Goal: start the lift, drive in right away, lower, release.
    chassis.turnToPoint(-38.24, -23.54, 1000, {.minSpeed = 20, .earlyExitRange = 5}, false);
    chassis.moveToPoint(-40.7, -23.54, 2000, {}, false);
    lift.LiftDown();
    pros::delay(600); // TUNE: how long the lift lowers.
    lift.LiftStop();
    openClaw();
    pros::delay(200);

    // Back up to x = -23.75, face 225, go to the corner Pin at (-46, -46): same sequence as the first cup.
    chassis.moveToPoint(-23.75, -23.54, 2000, {.forwards = false}, false);
    chassis.turnToHeading(225, 1000, {}, false);
    chassis.moveToPoint(-42, -42, 2000, {.maxSpeed = 40, .minSpeed = 30, .earlyExitRange = 1}, false); // 9 in short, chained
    closeClaw(); // Grab the pin.
    lift.LiftUpFor(500); // Lift rises during the next move; returns instantly.
    chassis.moveToPoint(-47, -46, 2000, {.maxSpeed = 62}, false);

    // // Face 0 and score in the upper red Goal from below, stopping 9 in short: start the lift, drive in, lower, release.
    chassis.turnToHeading(0, 1000, {.minSpeed = 20, .earlyExitRange = 5}, false);
    lift.LiftUpFor(500);
    chassis.moveToPoint(-47, -32.5, 2000, {}, false);
    lift.LiftDown();
    pros::delay(600); // TUNE: how long the lift lowers.
    lift.LiftStop();
    openClaw();
    pros::delay(200);
}

