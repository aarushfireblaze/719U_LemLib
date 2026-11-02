"""Host smoke test for the real opcontrol loop; hardware is replaced only in this test."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as directory:
    temp = Path(directory)
    (temp / 'main.h').write_text(r'''
#pragma once
#include <cassert>
struct Finished {};
inline int iteration = 0;
inline constexpr int samples[][2] = {{0, 0}, {127, -127}, {-127, 127}, {42, -23}};
namespace pros {
enum { E_CONTROLLER_MASTER, E_CONTROLLER_ANALOG_LEFT_Y, E_CONTROLLER_ANALOG_RIGHT_X,
       E_MOTOR_BRAKE_COAST, E_CONTROLLER_DIGITAL_R1, E_CONTROLLER_DIGITAL_R2 };
namespace lcd {
inline void set_text(int, const char*) {}
template<class... Args> void print(int, const char*, Args...) {}
}
struct Controller {
    explicit Controller(int id) { assert(id == E_CONTROLLER_MASTER); }
    int get_analog(int axis) {
        assert(axis == E_CONTROLLER_ANALOG_LEFT_Y || axis == E_CONTROLLER_ANALOG_RIGHT_X);
        return samples[iteration][axis == E_CONTROLLER_ANALOG_RIGHT_X];
    }
    bool get_digital(int button) {
        return button == E_CONTROLLER_DIGITAL_R1 ? (iteration == 1 || iteration == 3)
                                                : (iteration == 2 || iteration == 3);
    }
};
inline void delay(int ms) {
    assert(ms == 10);
    if (++iteration == 4) throw Finished{};
}
}
''')
    (temp / 'drivetrain.hpp').write_text(r'''
#pragma once
#include "main.h"
struct TestChassis {
    bool cancelled = false;
    bool coast = false;
    int calls = 0;
    void cancelAllMotions() { cancelled = true; }
    void setBrakeMode(int mode) { coast = mode == pros::E_MOTOR_BRAKE_COAST; }
    void arcade(int throttle, int turn) {
        assert(cancelled && coast);
        assert(throttle == samples[iteration][0]);
        assert(turn == -samples[iteration][1]);
        ++calls;
    }
};
inline TestChassis chassis;
''')
    (temp / 'lift.hpp').write_text(r'''
#pragma once
#include "main.h"
struct TestLift {
    int up = 0, down = 0, stop = 0;
    void LiftUp() { assert(iteration == 1); ++up; }
    void LiftDown() { assert(iteration == 2); ++down; }
    void LiftStop() { ++stop; }
};
inline TestLift lift;
''')
    (temp / 'test.cpp').write_text(r'''
#include "drivetrain.hpp"
#include "lift.hpp"
void opcontrol();
int main() {
    try { opcontrol(); } catch (const Finished&) {}
    assert(chassis.calls == 4);
    assert(lift.up == 1 && lift.down == 1 && lift.stop == 3);
}
''')
    compiler = shutil.which('c++')
    if not compiler:
        raise SystemExit('A host C++ compiler is required')
    executable = temp / 'test'
    subprocess.run([compiler, '-std=c++17', '-I', str(temp),
                    str(root / 'src/driver_control.cpp'), str(temp / 'test.cpp'),
                    '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
print('PASS: drivetrain inputs, R1/R2 holds, both/neither stop, coast, cancellation and 10 ms loop')
