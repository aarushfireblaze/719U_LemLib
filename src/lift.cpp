#include "lift.hpp"
#include "lift_feedback.hpp"
#include "pros/misc.hpp"

Lift lift;

namespace {
int maximumRpm(pros::v5::MotorGears gearing) {
    switch (gearing) {
        case pros::v5::MotorGears::red: return 100;
        case pros::v5::MotorGears::green: return 200;
        case pros::v5::MotorGears::blue: return 600;
        default: return 0;
    }
}
}

void Lift::Initialize() {
    if (task != nullptr) return; // Never re-zero on an autonomous/driver transition.
    bool ready = true;
    for (auto* motor : {&motor20, &motor5}) {
        if (motor->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES) != 1) ready = false;
        if (motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD) != 1) ready = false;
        if (motor->tare_position() != 1) ready = false;
    }
    rpm20 = maximumRpm(motor20.get_gearing());
    rpm5 = maximumRpm(motor5.get_gearing());
    Faulted.store(!ready || rpm20 == 0 || rpm5 == 0);
    Hold();
    task = new pros::Task([this] { Run(); });
}

// Commands are nonblocking; one worker owns all motor writes. Latest command wins.
void Lift::LiftUp() { stopAt.store(0); command.store(Command::Up); }
void Lift::LiftDown() { stopAt.store(0); command.store(Command::Down); }
void Lift::LiftStop() { stopAt.store(0); command.store(Command::Stop); }
// Deadline is set before the command so the worker never sees the new command with an old deadline.
void Lift::LiftUpFor(int ms) { stopAt.store(pros::millis() + ms); command.store(Command::Up); }
void Lift::LiftDownFor(int ms) { stopAt.store(pros::millis() + ms); command.store(Command::Down); }

void Lift::Hold() {
    if (holding) return; // Preserve the captured hold target while stationary.
    const auto first = motor20.brake();
    const auto second = motor5.brake();
    holding = first == 1 && second == 1;
    if (!holding) Faulted.store(true); // Retry braking next tick if communication failed.
}

void Lift::Run() {
    while (true) {
        const double first = motor20.get_position();
        const double second = motor5.get_position();
        Motor20Pos.store(first);
        Motor5Pos.store(second);
        if (std::isfinite(first) && std::isfinite(second))
            LiftPos.store(lift_feedback::position(first, second));
        EncoderMismatch.store(lift_feedback::mismatched(first, second));
        // Encoder disagreement is diagnostic only; transient motor lag must
        // not permanently disable the lift. Invalid feedback still stops it.
        if (!lift_feedback::valid(first, second)) Faulted.store(true);

        // A timed move whose time is up becomes a stop.
        auto deadline = stopAt.load();
        if (deadline != 0 && pros::millis() >= deadline && stopAt.compare_exchange_strong(deadline, 0))
            command.store(Command::Stop);
        const auto requested = command.load();
        const bool up = requested == Command::Up;
        const bool disabled = pros::competition::is_disabled();
        if (disabled) command.store(Command::Stop);
        if (disabled || Faulted.load() ||
            requested == Command::Stop) {
            Hold();
        } else {
            holding = false;
            // Lowering runs at 60% of max speed; raising stays at full speed.
            const auto a = motor20.move_velocity(up ? rpm20 : -rpm20 * 60 / 100);
            const auto b = motor5.move_velocity(up ? rpm5 : -rpm5 * 60 / 100);
            if (a != 1 || b != 1) {
                Faulted.store(true);
                Hold();
            }
        }
        pros::delay(5);
    }
}
