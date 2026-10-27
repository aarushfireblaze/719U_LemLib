#include "main.h"
#include "drivetrain.hpp"

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    chassis.cancelAllMotions();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    while (true) {
        const int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(throttle, turn);
        pros::delay(25);
    }
}
