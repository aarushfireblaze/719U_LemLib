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
    // Field coordinates in inches, field center = (0, 0).
    // Robot starts a little in front of the toggle, facing +x: back up 5 in fast,
    // then call that spot (-63.96, 0): path.jerryio's start point (-162.46 cm), left wall, center line.
    chassis.setPose(0, 0, 90);
    chassis.moveToPoint(-5, 0, 1000, {.forwards = false, .minSpeed = 100, .earlyExitRange = 0.5}, false);
    chassis.setPose(-61.2, 0, 90);

    chassis.moveToPoint(-46, 0, 1000, {.forwards = false, .minSpeed = 100, .earlyExitRange = 0.5}, false);
    // chassis.moveToPoint(l)
    // Drive to the center-line Pin, stopping at x = -80 cm (-31.5 in), short of the Pin at -23.55 in.
    // Normal speed most of the way, then the last 12 in very slowly so the Pin does not tip.
    // chassis.moveToPoint(-43.5, 0, 3000, {.minSpeed = 20, .earlyExitRange = 1}, false);
    // openClaw(); // Getting close: open before the slow approach.
    // chassis.moveToPoint(-28.7, 0, 3000, {.maxSpeed = 20}, false);
    // closeClaw(); // On the spot: close around the Cup.
    // pros::delay(200); // Let the piston grip before lifting.
    // lift.LiftUp();
    // pros::delay(600); // TUNE: how long the lift raises.
    // lift.LiftStop();  // Hold the lift up while driving.

    // // Upper red Goal at (-47.1, -23.55): face it, then drive straight at it at normal speed.
    // // Stops ~7 in short (claw reach ~5 in, matching the Pin stop, + 2 in so it does not ram the Goal).
    // chassis.turnToPoint(-47.1, -23.55, 1500, {}, false);
    // chassis.moveToPoint(-42.7, -17.9, 3000, {}, false);

    // // At the Goal: lower the lift, wait half a second, then release.
    // lift.LiftDown();
    // pros::delay(600); // TUNE: how long the lift lowers (down runs at 60% speed).
    // lift.LiftStop();
    // pros::delay(500);
    // openClaw();
}

// Reverse-swing test for finding a swing start point (not called; swap into autonomous()).
// Push the robot into the matchloader facing 90, press A, read X / Y on the controller.
void reverseSwingTest() {
    chassis.setPose(0, 0, 0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    pros::lcd::set_text(7, "Push into matchloader, press A");
    while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) pros::delay(10);
    chassis.swingToHeading(0, lemlib::DriveSide::RIGHT, 3000,
                           {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    const auto pose = chassis.getPose();
    pros::lcd::print(5, "Swing start: X %.2f Y %.2f", pose.x, pose.y);
}
