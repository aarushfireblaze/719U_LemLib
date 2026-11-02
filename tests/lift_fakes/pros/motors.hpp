#pragma once
#include <array>
#include <cstdlib>
namespace pros {
namespace v5 { enum class MotorGears { invalid, red, green, blue }; enum class MotorUnits { degrees }; }
constexpr int E_MOTOR_ENCODER_DEGREES = 0;
constexpr int E_MOTOR_BRAKE_HOLD = 1;
struct FakeMotor {
    double position = 0, target = 0;
    int port = 0, units = -1, brakeMode = -1, zeroes = 0, moves = 0, brakes = 0;
    int result = 1, brakeResult = 1, rpm = 0;
    v5::MotorGears gearing = v5::MotorGears::green;
};
inline std::array<FakeMotor, 22> motors;
class Motor {
    int port;
public:
    Motor(int p, v5::MotorGears, v5::MotorUnits) : port(std::abs(p)) { motors[port].port = p; }
    int set_encoder_units(int units) { motors[port].units = units; return motors[port].result; }
    int set_brake_mode(int mode) { motors[port].brakeMode = mode; return motors[port].result; }
    int tare_position() { ++motors[port].zeroes; motors[port].position = 0; return motors[port].result; }
    double get_position() { return motors[port].position; }
    v5::MotorGears get_gearing() { return motors[port].gearing; }
    int brake() { ++motors[port].brakes; return motors[port].brakeResult; }
    int move_velocity(int rpm) {
        motors[port].rpm = rpm;
        ++motors[port].moves; return motors[port].result;
    }
};
}
