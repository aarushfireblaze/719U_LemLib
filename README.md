# 719 LemLib template

PROS 4.2.2 / LemLib 0.5.6. Uses LemLib's drivetrain, odometry, drive curves,
and arcade control APIs directly.

## Where to edit

| File | Purpose |
| --- | --- |
| `include/robot_config.hpp` | Motor/sensor ports, cartridge, wheel sizes, gearing, tracking wheel offset, joystick curve |
| `src/drivetrain.cpp` | Hardware objects and LemLib drivetrain/odometry/chassis construction |
| `include/pid_tuning.hpp` | Lateral and angular LemLib PID settings |
| `include/drivetrain.hpp` | Shared chassis declaration |
| `src/driver_control.cpp` | Driver control (`opcontrol`) |
| `src/main.cpp` | Initialization, brain pose display, competition callbacks, autonomous |

## Configure before driving

The existing values are retained as starting values, not verified robot measurements:

- Left motors: **-5, 4, -3**. Right motors: **6, -9, 7**. Negative ports reverse motors.
  Change the lists in `robot_config.hpp`; you can add/remove motors there.
  Every motor must move its side forward with positive power. Smart ports are 1–21;
  do not assign the same physical port to multiple devices.
- Blue **600 RPM** cartridges, **360 RPM at the wheels**, new **4-inch** drive wheels,
  **10-inch** track width. Wheel RPM = cartridge RPM × driving gear teeth / driven
  gear teeth; use 600 for direct drive with blue cartridges. Measure track width
  between the centers of the left and right wheels.
- IMU: port **10**. Forward/backward tracking wheel: reversed rotation sensor on
  port **11**, new **2.75-inch** wheel, offset **0 inches**. Set the measured offset:
  positive to the right of the robot's tracking center, negative to the left.
  A forward roll should increase this wheel's measured distance.
- No horizontal tracking wheel is configured. LemLib fills the unused second
  vertical wheel with drive motor encoders. If there is no vertical tracking wheel
  on your robot, replace `&verticalWheel` with `nullptr` in `OdomSensors` and remove
  the unused rotation sensor/tracking wheel objects. LemLib then uses motor encoders
  for forward distance; slip will affect accuracy. A missing configured sensor is
  not equivalent to `nullptr`.

Keep the robot still during startup IMU calibration. Start with the wheels raised
to check motor directions, then check forward motion and clockwise turning at low
stick input. Driver controls are **left stick Y** for forward/backward and **right
stick X** for turning, updated every 25 ms using `chassis.arcade()`. LemLib applies
the configured deadband and exponential curves. Driver control uses coast braking.

Autonomous resets the pose and drives forward 24 inches, with a 3000 ms timeout.
The retained PID/drift values require tuning on the actual robot. Without a horizontal tracking wheel,
sideways slip is not measured. The brain display reports odometry, not proof that
sensor calibration succeeded; inspect the hardware and LemLib warnings if values
are wrong.

## PID settings

Edit the lateral and angular `lemlib::ControllerSettings` in
`include/pid_tuning.hpp`, then rebuild and upload. These settings are passed
directly to the chassis. Driver control is simple arcade: left stick Y drives
forward/backward, and right stick X turns. There are no PID test button controls.

Run autonomous to test the 24-inch forward motion in `src/main.cpp`. Each run
resets the starting pose to `(0, 0, 0)`. Keep the robot's path clear.

## Build

Run `pros make` from this directory with the PROS CLI and ARM toolchain installed,
or use the PROS VS Code extension's build command. Plain `make` also works when
the PROS ARM toolchain is on `PATH`.

## Driver control smoke test

Run `python3 tests/test_driver_control.py` with a host C++ compiler installed.
The test compiles the actual driver loop against test doubles and checks neutral,
full-scale, and mixed joystick inputs, motion cancellation, coast braking, and the
25 ms update delay. It does not simulate LemLib internals or robot hardware.

On this Mac, the default host linker rejects the macOS 27 SDK. The test passed with:
`SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX14.4.sdk python3 tests/test_driver_control.py`.
The PROS ARM build does not need this workaround.

## Official references

- [LemLib configuration](https://lemlib.readthedocs.io/en/stable/tutorials/2_configuration.html)
- [LemLib driver control](https://lemlib.readthedocs.io/en/stable/tutorials/3_driver_control.html)
- [LemLib PID tuning](https://lemlib.readthedocs.io/en/stable/tutorials/4_pid_tuning.html)
- [LemLib 0.5.6 source](https://github.com/LemLib/LemLib/tree/v0.5.6)
