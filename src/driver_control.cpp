#include "main.h"
#include "drivetrain.hpp"
#include "lift.hpp"
#include "claw.hpp"

void opcontrol() {
    pros::lcd::set_text(6, "Mode: DRIVER starting");
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    chassis.cancelAllMotions();
    lift.LiftStop();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::lcd::set_text(6, "Mode: DRIVER active");

    while (true) {
        const int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int turn = -controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // Bias 1.0 favors turning when throttle + turn exceed motor power: tighter turns at full speed.
        chassis.arcade(throttle, turn, false, 1.0);
        const bool up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        const bool down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        static unsigned int displayTicks = 0;
        if (++displayTicks % 10 == 0)
            pros::lcd::print(7, "Stick %d/%d Lift %d/%d", throttle, turn, up, down);
        if (up == down) lift.LiftStop(); // Both held or both released: hold.
        else if (up) lift.LiftUp();
        else lift.LiftDown();
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            setClaw(!clawExtended); // Y toggles the claw piston.
        }
        pros::delay(10);
    }
}
