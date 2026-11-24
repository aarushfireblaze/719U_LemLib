#include "main.h"
#include "drivetrain.hpp"
#include "lift.hpp"
#include "claw.hpp"

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    chassis.cancelAllMotions();
    lift.LiftStop();
    setAutoClamp(true); // TESTING: auto-clamp on in driver. Set back to false after testing.
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    while (true) {
        const int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int turn = -controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // Bias 0.7 leans toward turning when throttle + turn exceed motor power: tighter turns at full speed.
        chassis.arcade(throttle, turn, false, 0.7);
        const bool up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        const bool down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        if (up == down) lift.LiftStop(); // Both held or both released: hold.
        else if (up) lift.LiftUp();
        else lift.LiftDown();
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            setClaw(!clawExtended.load()); // Y toggles the claw piston.
        }
        pros::delay(10);
    }
}
