# Four-bar lift: unrestricted travel

Motor 20 is normal; motor 5 is reversed. The shared lift instance is available
in autonomous and driver control. LiftUp() and LiftDown() run continuously in
the requested direction at maximum configured RPM. LiftStop() stops and holds.
Commands return immediately; the latest command wins. A single background
worker owns motor writes and updates both encoders every 5 ms, plus scheduling
and device latency. Driver buttons are sampled every 10 ms.

R1 raises; R2 lowers. Both or neither held stops and holds. Disable clears the
motion command. Software holding torque requires power and an enabled robot.

There are NO software travel limits, endpoint targets, endpoint slowing, or
position clamping. The previous 0-900 degree limits and buffers were removed
at the user's request. Holding a button continues motion until released or a
fault occurs.

Initialize() zeroes both encoders once at startup; mode changes do not re-zero.
LiftPos.load() is the unrestricted average in motor-shaft degrees. Motor20Pos
and Motor5Pos expose raw feedback. Values can be negative or exceed 900.
Separate atomic attributes are not one simultaneous snapshot. Invalid feedback
preserves the last finite average and latches Faulted.

Configured red/green/blue gearing selects 100/200/600 RPM. Firmware gearing must
match the physical cartridge; cartridge color is not physically detected.
Invalid gearing, motor command errors, or nonfinite feedback stop both motors
and latch Faulted until restart. Encoder disagreement above 15 degrees sets
EncoderMismatch and a display warning; it does not stop motion.

Host checks: bash tests/run_lift_tests.sh and python3 tests/test_driver_control.py.
These exercise the real worker with fake devices and driver button mapping,
including unrestricted positive/negative travel, speed, reversal, hold, disable,
and faults. They do not prove hardware direction, clearance or holding force.
