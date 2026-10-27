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
       E_MOTOR_BRAKE_COAST };
struct Controller {
    explicit Controller(int id) { assert(id == E_CONTROLLER_MASTER); }
    int get_analog(int axis) {
        assert(axis == E_CONTROLLER_ANALOG_LEFT_Y || axis == E_CONTROLLER_ANALOG_RIGHT_X);
        return samples[iteration][axis == E_CONTROLLER_ANALOG_RIGHT_X];
    }
};
inline void delay(int ms) {
    assert(ms == 25);
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
        assert(turn == samples[iteration][1]);
        ++calls;
    }
};
inline TestChassis chassis;
''')
    (temp / 'test.cpp').write_text(r'''
#include "drivetrain.hpp"
void opcontrol();
int main() {
    try { opcontrol(); } catch (const Finished&) {}
    assert(chassis.calls == 4);
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
print('PASS: neutral, full-scale and mixed joystick inputs; coast, cancellation and 25 ms loop')
