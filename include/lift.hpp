#pragma once

#include <atomic>
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

// Positions are motor-shaft degrees relative to the position at Initialize().
// Initialize only once; the startup position becomes encoder zero.
class Lift {
public:
    void Initialize();
    void LiftUp();
    void LiftDown();
    void LiftStop();

    // Refreshed every 5 ms. LiftPos is the unbounded average of both encoders.
    std::atomic<double> LiftPos{0};
    std::atomic<double> Motor20Pos{0};
    std::atomic<double> Motor5Pos{0};
    std::atomic<bool> Faulted{false};
    std::atomic<bool> EncoderMismatch{false}; // Diagnostic only; does not stop motion.

private:
    enum class Command { Stop, Up, Down };
    void Run();
    void Hold();
    pros::Motor motor20{20, pros::v5::MotorGears::invalid, pros::v5::MotorUnits::degrees};
    pros::Motor motor5{-5, pros::v5::MotorGears::invalid, pros::v5::MotorUnits::degrees};
    std::atomic<Command> command{Command::Stop};
    bool holding = false; // Only Initialize/worker access motor control state.
    int rpm20 = 0;
    int rpm5 = 0;
    pros::Task* task = nullptr;
};

extern Lift lift;
