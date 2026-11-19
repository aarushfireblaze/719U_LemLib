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
    // Robot starts 5 in in front of the toggle on the center line, facing +x, then backs up fast to (-61.2, 0).
    chassis.setPose(-56.2, 0, 90);
    chassis.moveToPoint(-61.2, 0, 1000, {.forwards = false, .minSpeed = 100, .earlyExitRange = 0.5}, false);

    // Path from path.jerryio (inches). Each point is a straight line; turns happen in place first.
    // 1. Forward to (-46.73, -0.05).
    chassis.moveToPoint(-46.73, -0.05, 2000, {}, false);
    // 2. Turn to face down, forward to (-46.73, -14.05).
    chassis.turnToPoint(-46.73, -14.05, 1000, {.minSpeed = 20, .earlyExitRange = 5}, false); // chained into the drive
    chassis.moveToPoint(-46.73, -14.05, 2000, {}, false);
    // 3. Back straight up to (-46.73, -0.25).
    chassis.moveToPoint(-46.73, -0.25, 2000, {.forwards = false}, false);
    // 4. Turn to face +x, forward to (-32.73, -0.25).
    chassis.turnToPoint(-32.73, -0.25, 1000, {}, false);
    chassis.moveToPoint(-32.73, -0.25, 2000, {}, false);
    // 5. Slowly to the center-line Pin spot (-24.26, -0.25).
    chassis.moveToPoint(-24.26, -0.25, 2000, {.maxSpeed = 30}, false);
    // 6. Turn toward (-40.23, -16.61) (about 225 deg), forward to it.
    chassis.turnToPoint(-40.23, -16.61, 1000, {.minSpeed = 20, .earlyExitRange = 5}, false); // chained into the drive
    chassis.moveToPoint(-40.23, -16.61, 2000, {}, false);
    // 7. Back straight up to (-23.92, -0.03).
    chassis.moveToPoint(-23.92, -0.03, 2000, {.forwards = false}, false);
    // 8. Turn to face down, forward to (-23.75, -14.35).
    chassis.turnToPoint(-23.75, -14.35, 1000, {}, false);
    chassis.moveToPoint(-23.75, -14.35, 2000, {}, false);
    // 9. Slowly to the middle Pin spot (-23.75, -23.19).
    chassis.moveToPoint(-23.75, -23.19, 2000, {.maxSpeed = 30}, false);
    // 10. Turn to face -x (about 270 deg), forward to (-38.24, -23.54) at the red Goal.
    chassis.turnToPoint(-38.24, -23.54, 1000, {.minSpeed = 20, .earlyExitRange = 5}, false); // chained into the drive
    chassis.moveToPoint(-38.24, -23.54, 2000, {}, false);

    pros::lcd::print(7, "Auton ended: %lu ms",
                     static_cast<unsigned long>(pros::millis() - started));
}

