// Runs the real src/lift.cpp worker one iteration at a time, using fake devices.
#include "lift.hpp"
#include "pros/misc.hpp"
#include <cassert>
#include <limits>

void tick() {
    try { pros::worker(); } catch (const pros::TickFinished&) { return; }
    assert(false);
}
int main(int argc, char** argv) {
    auto& a = pros::motors[20];
    auto& b = pros::motors[5];
    const int variant = argc > 1 ? std::atoi(argv[1]) : 2;
    a.gearing = b.gearing = static_cast<pros::v5::MotorGears>(variant);
    lift.Initialize();
    if (variant == 0) {
        lift.LiftUp(); tick();
        assert(lift.Faulted.load() && a.moves == 0 && b.moves == 0);
        return 0;
    }
    assert(a.port == 20 && b.port == -5);
    assert(a.brakeMode == pros::E_MOTOR_BRAKE_HOLD && b.brakeMode == a.brakeMode);
    lift.Initialize();
    assert(a.zeroes == 1 && b.zeroes == 1);
    lift.LiftDown(); tick();
    assert(a.rpm < 0 && b.rpm < 0); // No lower travel limit.
    lift.LiftUp(); tick();
    const int expectedRpm = variant == 1 ? 100 : variant == 2 ? 200 : 600;
    assert(a.rpm == expectedRpm && b.rpm == expectedRpm);
    a.position = 100; b.position = 110;
    lift.LiftDown(); tick();
    assert(a.rpm == -expectedRpm && b.rpm == -expectedRpm && lift.LiftPos.load() == 105);
    lift.LiftUp(); lift.LiftDown(); lift.LiftStop();
    const int count = a.moves;
    tick(); tick();
    assert(a.moves == count && b.moves == count); // Latest command wins.
    const int brakes = a.brakes;
    tick();
    assert(a.brakes == brakes); // Holding target is not recaptured repeatedly.
    a.position = 1800; b.position = 1810;
    lift.LiftUp(); tick();
    assert(a.moves == count + 1 && b.moves == count + 1 && !lift.Faulted.load());
    assert(lift.LiftPos.load() == 1805);
    a.position = -100; b.position = -110;
    lift.LiftDown(); tick();
    assert(a.moves == count + 2 && b.moves == count + 2 && !lift.Faulted.load());
    assert(lift.LiftPos.load() == -105);
    a.position = b.position = 100;
    lift.LiftUp(); tick();
    pros::competition::disabled = true; tick();
    const int beforeEnable = a.moves;
    pros::competition::disabled = false; tick();
    assert(a.moves == beforeEnable); // Disabled commands cannot resume on enable.
    lift.LiftDown(); tick();
    b.position = 116; tick();
    assert(!lift.Faulted.load() && lift.EncoderMismatch.load());
    const int beforeRecovery = a.moves;
    b.position = 100; lift.LiftUp(); tick();
    assert(a.moves == beforeRecovery + 1 && !lift.EncoderMismatch.load());
    b.position = std::numeric_limits<double>::infinity(); tick();
    assert(lift.Faulted.load());
    const int beforeFault = a.moves;
    b.position = 100; lift.LiftUp(); tick();
    assert(a.moves == beforeFault); // Fault remains latched after readings recover.
    a.position = b.position = 901; tick();
    assert(lift.LiftPos.load() == 901 && lift.Motor20Pos.load() == 901);
    b.position = std::numeric_limits<double>::infinity(); tick();
    assert(lift.LiftPos.load() == 901);
}
