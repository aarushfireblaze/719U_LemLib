# Tuning guide: IMU scale and EKF

Do these steps **in order**. Each one depends on the ones before it:

- Wrong heading aims every sensor beam wrong.
- Wrong odometry or sensor offsets make every expected distance wrong.
- MAX_RESIDUAL depends on Q's drift test.

All values live in `include/localization/ekf_config.hpp`.

| Step | What | Section in config |
|---|---|---|
| 1 | `IMU_SCALE` | SETUP |
| 2 | Tracking wheel check | `robot_config.hpp` |
| 3 | Sensor ports, offsets, walls, toggle, `R_BIAS_IN` | SETUP |
| 4 | `MAX_ANGLE_DEG` | TUNING |
| 5 | `R` | TUNING |
| 6 | `Q` (drift test) | TUNING |
| 7 | `MAX_RESIDUAL` | TUNING |
| 8 | Fine-tune `Q` | TUNING |
| 9 | Final checks | — |

Never touch the DEFAULTS section. Where to read numbers (controller pages, terminal log) is at the
bottom.

**Controller pages:** press **X** to cycle: main → IMU → FRONT → BACK → LEFT → RIGHT → main.

**TEMP snippet** (used in steps 3–5): starts the EKF without letting it correct, so you can read
sensors at a known pose. Put it at the top of `opcontrol()` and remove it when done:

```cpp
localizer.start({-60.2, 0, 270}); // TEMP: known pose (safe even if auton already started the EKF)
localizer.setEnabled(false);      // TEMP: read sensors only, don't correct
```

---

## Step 1: IMU_SCALE

**What it does:** LemLib's heading = raw IMU rotation × `IMU_SCALE`. Most IMUs read slightly more
or less than 360° per real turn, for example 358° or 361.5°. That error adds up over a match.

**Check calibration first.** Power on with the robot completely still and flat. Startup waits
until the IMU has really finished calibrating, about 2–3 seconds.
- If controller line 1 says `IMU CAL FAILED!`, power-cycle.
- If it keeps happening, check the IMU cable and port.

**Measure:**
1. Press **X** once to get to the **IMU page**. `raw:` is the IMU's total rotation, unscaled. It
   keeps counting past 360, so there's nothing to wrap-count, and it works whatever `IMU_SCALE` is
   currently set to.
2. Push the robot's back flat against a field wall. Write down `raw:`. This is the **start** value.
3. Spin the robot **10 full turns clockwise**, by hand or driving. Smooth and not too fast is best.
4. Push the back flat against the **same wall** again. Write down `raw:`. This is the **end**
   value.
5. Work out the scale:
   ```
   scale_cw = 3600 / (end − start)
   ```
6. Repeat steps 2–5 spinning **counterclockwise**. Use the absolute value of `end − start`.
7. Average the two: `IMU_SCALE = (scale_cw + scale_ccw) / 2`. Round to 4 decimals.

**Example:** clockwise reads 3589 → 1.0031. Counterclockwise reads 3591 → 1.0025. So
`IMU_SCALE = 1.0028`.

**Test it:** upload and repeat the 10 turns. The `scl:` line should now change by 3600 ± 3 both
ways. A value outside 0.9–1.1 won't compile, which means a measuring mistake.

---

## Step 2: Tracking wheel check

**Why:** the EKF only corrects drift. If the tracking wheel reads distance wrong, odometry is
wrong all the time, and the EKF ends up fighting it.

1. Put the robot on a tile seam facing straight along it (heading 0). Restart the program, so the
   pose starts at 0, 0.
2. **Push the robot straight forward exactly 48"** (two tiles) along the seam, by hand and without
   turning it.
3. Main page line 1 (`oX: Y:`) should show **Y ≈ 48.0** (±0.5) and X ≈ 0.
   - **Y reads negative:** the tracking wheel direction is flipped. Flip the sign of
     `verticalEncoderPort` in `robot_config.hpp`.
   - **Y is off by more than 0.5":** fix the wheel size in `robot_config.hpp`:
     ```
     new trackingWheelDiameter = old × 48 / (Y reading)
     ```
4. Repeat 3 times. The readings should agree within about 0.3".

---

## Step 3: Sensor setup (ports, offsets, walls, toggle, bias)

These are measurements, not tuning. Get them right before touching the tuning values.

**Measure and fill in (SETUP section):**
- **Ports:** each sensor's real smart port.
- **Offsets:** tape-measure from the robot's tracking center to each sensor's **face** (where the
  beam leaves). `offsetX` is positive to the right, `offsetY` positive forward.
- **Mount angles:** front 0, back 180, left -90, right 90.
- **Walls:** measure from the inside face of the left wall to the inside face of the right wall,
  then halve it. That's `WALL_X`. Do the same top to bottom for `WALL_Y`.
- **Toggle:** its width along the wall and how far it sticks out from the wall face.

**Static check.** Use the TEMP snippet, with the pose you'll place the robot at. Run driver
control with a laptop logging (`pros terminal > log.csv`).
1. Place the robot exactly at that pose.
2. Every sensor facing a wall should show `_meas` within **1"** of `_exp`.
   - Off by about the same amount at every distance: fix that sensor's offset.
   - Fine close to a wall but worse farther away: the mount angle or heading is wrong. Check the
     sensor is mounted straight, and that step 1 is done.
3. Check 2–3 more known spots, tape-measured from the walls. Update the pose in the snippet each
   time. **Include spots where each sensor is about 42" from its wall.**

**Set `R_BIAS_IN`:** after the offsets are right, take the **largest |`_meas` − `_exp`|** you saw
at the ~42" spots. Put it in `R_BIAS_IN`. The code never uses less than 0.3". This covers the small
leftover offset, angle and wall errors that noise alone doesn't.

---

## Step 4: MAX_ANGLE_DEG

**What it does:** throws out readings where the beam hits the wall at a steep angle. The beam is a
cone, so at steep angles the near edge returns first and the reading comes out short.

1. Use the TEMP snippet with a laptop logging. Temporarily set `MAX_ANGLE_DEG = 60` so nothing is
   rejected for angle.
2. Put the robot so one sensor is about **42"** from a wall and square to it. `_meas` should match
   `_exp`.
3. Rotate the robot in 5° steps. Update the snippet's heading each time, or restart at each angle.
   Compare `_meas` to `_exp`.
4. Find the angle where `_meas` starts reading more than about **0.5" shorter** than `_exp`.
5. Set `MAX_ANGLE_DEG` **5° below** that angle. Expect somewhere around 20–30°.
6. Optional: repeat at the farthest wall distance you usually see in a match. If that gives a
   smaller angle, use the smaller one.

Set it once and leave it.

---

## Step 5: R (sensor noise)

**What it does:** tells the EKF how much to trust the distance sensors. R is defined at **42"**.
The code makes it bigger for farther walls and smaller for closer ones.

**Formula (computed on the robot):**
```
R = (std dev + R_BIAS_IN)²     then converted to 42"
```
- **std dev** is the true standard deviation of each sensor's last 75 fresh readings, about 6 s.
  Readings are spaced at least 60 ms apart so no reading is ever counted twice. A repeated reading
  would make the noise look smaller than it is.
- **R_BIAS_IN** comes from step 3.

**Sensor pages:** use the TEMP snippet and press **X** to reach a sensor page:
```
BACK  n: 75/ 75        how full the window is
sd0.172 b0.30          std dev and bias (inches)
d 42.1 R: 0.223        average distance, and the R for this sensor
```
Line 3 shows `wait...` until the window is full.

**Measure, two sensors at a time, using two corners:**
1. **Corner 1:** put the robot in a corner so BACK and LEFT each have their face about **42"** from
   their own wall, square to it. Use corners away from the toggle.
2. Don't touch the robot. Wait until `n: 75/ 75`, then about 6 more seconds so the window holds
   only still readings.
3. Write down the `sd` and `R` for BACK, then page to LEFT and do the same.
4. **Corner 2:** turn the robot 180° in the opposite corner for FRONT and RIGHT. Repeat steps 2–3.

**Pick R:**
- **All four `sd` within about 0.2" of each other:** set the global `R` to the **biggest** of the
  four R values.
- **One sensor clearly worse:** find the cause first.
  - loose mount
  - tilt (beam hitting the floor or the top of the wall)
  - a robot part or cable in the beam's cone
  - a shiny spot on the wall

  If it's still noisy after that, give that sensor its own `r` in SENSORS, and use the biggest of
  the other three for `R`.

**Example:** `sd` 0.17, bias 0.30 → R = 0.47² ≈ 0.22.

---

## Step 6: Q (odometry drift test)

**What it does:** tells the EKF how fast odometry drifts per inch driven.
- Higher Q trusts the sensors more and corrects faster.
- Lower Q trusts odometry more and corrects more smoothly.

1. Set `EKF_ENABLED = false`. Controller line 1 now shows `oX: Y:`, which is pure odometry.
2. Run your real auton. Add up roughly how many inches it drives in total. That's `D`.
3. At the end, tape-measure the robot's real position and compare it to `oX`/`Y`. The
   straight-line distance between them is the drift error **`e`**. **Write `e` down: step 7 needs
   it.**
4. Set:
   ```
   Q = e² / D
   ```
5. Do 2–3 runs and use the **worst** (largest) `e`.

**Example:** 3" off after about 200" driven → Q = 9 / 200 = 0.045.

---

## Step 7: MAX_RESIDUAL

**What it does:** a reading more than this far from the expected distance is treated as blocked
(by a robot or game element) and ignored.

**Why it has to be big enough:** if odometry drifts further from the truth than MAX_RESIDUAL, then
*every* correct reading looks "blocked" and gets rejected. With no readings accepted, nothing
corrects the drift, so it keeps growing and the EKF is **locked out for the rest of the match**. So
MAX_RESIDUAL must be bigger than how far odometry can drift on its own, which is `e` from step 6.

**How to set it:**
1. Set `EKF_ENABLED = true`. Drive around an **empty** field for about a minute with a laptop
   logging. Look at **every** reading, `OK` and `RES`. On an empty field nothing blocks the beam, so
   any gap is sensor and model error. Note the **biggest |`_meas` − `_exp`|**.
2. Set:
   ```
   MAX_RESIDUAL = larger of:
       2–3 × (biggest empty-field gap from step 1)
       3 × e from step 6, and at least 3"
   ```

**Example:** biggest empty-field gap 1.2" → 2.4–3.6. Drift `e` = 3" → 9. Use **9**.

**Check it:** hold a box in front of one sensor. It should switch to `N` on the controller (`RES`
in the log), and the pose shouldn't jump.

---

## Step 8: Fine-tune Q

With the EKF on, run autons and adjust **only Q**, in factors of 2:

| Symptom | Fix |
|---|---|
| Pose jitters, or `moveToPoint` wobbles at the end of moves | Q ÷ 2 |
| Pose still drifts, or corrects slowly (`odom` and `ekf` log columns barely differ) | Q × 2 |

If you raise Q a lot, recheck that MAX_RESIDUAL is still at least 3 × `e`.

---

## Step 9: Final checks

1. **EKF off = old robot.** Set `EKF_ENABLED = false` and run an auton. It must drive exactly as
   before the EKF existed, apart from the IMU scale and any tracking wheel fix.
2. **Push test.** Robot still, EKF on: push it a few inches sideways by hand. Controller X/Y should
   move to the true position within about a second, smoothly, in the right direction.
3. **Auton comparison.** Run the same auton 3 times with the EKF on and 3 times off. Tape-measure the
   final error each time. EKF on should be clearly better.
4. **Code tests.** After changing anything in `src/localization/`, run `tests/run_ekf_tests.sh` on
   a laptop.

---

## Reference: reading the numbers

**Brain screen:** nothing prints there.

**Controller** (`CONTROLLER_TELEMETRY` in DEFAULTS). **X** cycles the pages.

**Main page**
- **Line 1:**
  - `X: Y:` is the EKF position, shown while the EKF is correcting.
  - `oX: Y:` is pure odometry, shown whenever the EKF is disabled, not started, or set to
    `setEnabled(false)`.
  - `IMU CAL FAILED!` overrides both.
- **Line 2:**
  - `F:Y B:N L:Y R:Y` shows whether each sensor's last reading was used. It ends in `off` when
    `setEnabled(false)`.
  - Otherwise `EKF not started` or `EKF disabled`.
- **Line 3:**
  - `H:` is the heading, always shown.
  - `SD:` is the position uncertainty in inches, shown while the EKF runs.

**IMU page**
- `raw:` is the unscaled total rotation.
- `scl:` is raw × `IMU_SCALE`, which is what LemLib uses.
- Both are unwrapped.

**Sensor pages** (FRONT, BACK, LEFT, RIGHT)
- Line 1: readings in the window.
- Line 2: `sd` (std dev) and `b` (bias).
- Line 3: `d` (average distance) and `R`.

**Laptop terminal** (`TERMINAL_TELEMETRY` in DEFAULTS): run `pros terminal > log.csv`. You get one
line every 100 ms, starting with `ekf,`:

| Column | Meaning |
|---|---|
| `ms` | Time |
| `ekf_x`, `ekf_y` | Corrected position |
| `odom_x`, `odom_y` | LemLib position before correction |
| `std` | Position uncertainty (inches) |
| `<SENSOR>_meas` / `_exp` | Measured / expected distance (`-1` = none) |
| `<SENSOR>_r` | Effective R used for that reading |
| `<SENSOR>_status` | `OK` or a reject code (below) |

**Status codes:**

| Code | Meaning |
|---|---|
| `OK` | Reading used |
| `RES` | Too far from expected (beam blocked), see step 7 |
| `ANG` | Beam hits the wall too steeply, see step 4 |
| `RNG` | Outside the sensor's range (under 20 mm or over 2 m) |
| `CONF` | Sensor unsure of its own reading |
| `NONE` | Nothing seen, or sensor unplugged |
| `MISS` | Beam misses every wall in the map, so check the walls in step 3 |
| `OFF` | Sensor disabled in config |
| `OLD` | Waiting for the sensor's next sample |
