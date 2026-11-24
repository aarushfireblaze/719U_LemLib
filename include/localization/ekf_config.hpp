#pragma once

// EKF position correction on top of LemLib odometry. See EKF_TUNING.md.
// This header is PROS-free so the off-robot tests can include it.

#include <array>
#include <cstdint>

namespace ekf_config {

// ===================== EKF TUNING (the only values we tune) =====================
inline constexpr double Q = 0.02;            // process noise (in^2 per inch traveled): how much to distrust
                                             //   odometry. Higher = trust sensors more, corrections faster.
inline constexpr double R = 1.0;             // measurement noise (in^2) at REF_DISTANCE_IN (42"): how much to
                                             //   distrust the distance sensors. Scaled by (distance/42)^2.
                                             //   Higher = smoother, slower corrections.
                                             //   Measure: R = (std dev + R_BIAS_IN)^2, read off the
                                             //   controller sensor pages (EKF_TUNING.md step 5).
inline constexpr double MAX_RESIDUAL = 4.0;  // inches. Reject a reading if |measured - expected| > this
                                             //   (a robot or game element is blocking the beam).
inline constexpr double MAX_ANGLE_DEG = 25.0; // Reject a reading if the beam hits the wall more than this
                                             //   many degrees away from perpendicular.

// ===================== SETUP (measure once) =====================
// IMU scale: LemLib sees rotation = raw IMU rotation * IMU_SCALE. Fixes an IMU that reads a bit
// more or less than 360 per real turn. 1.0 = no correction. Applies even when EKF_ENABLED is false.
// Measure: IMU_SCALE = (360 * turns) / (raw degrees read). See EKF_TUNING.md step 1.
inline constexpr double IMU_SCALE = 1.0;

// Sensor bias for the R formula: the largest |measured - expected| seen at ~42" during the step 3
// static check, in inches. Covers small offset/angle/wall errors that noise alone doesn't.
// Never below MIN_R_BIAS_IN (enforced in code).
inline constexpr double R_BIAS_IN = 0.3;

inline constexpr bool EKF_ENABLED = true; // false = robot behaves exactly as before the EKF existed

// Per-sensor override value meaning "use the global tuning value above".
// Q has no per-sensor override: it describes odometry, not a sensor.
inline constexpr double USE_GLOBAL = -1.0;

struct SensorSetup {
    const char* name;
    bool enabled;
    std::uint8_t port;
    double offsetX;       // inches from tracking center to the sensor FACE, + = right of center
    double offsetY;       // inches from tracking center to the sensor FACE, + = forward of center
    double mountAngleDeg; // beam direction relative to robot front, clockwise + (right = 90)
    double r = USE_GLOBAL;
    double maxResidual = USE_GLOBAL;
    double maxAngleDeg = USE_GLOBAL;
};

// TODO: placeholder ports and offsets. Measure on the robot and replace.
inline constexpr std::array<SensorSetup, 4> SENSORS = {{
    {"FRONT", true, 4, 0.0, 6.0, 0.0},
    {"BACK", true, 6, 0.0, -6.0, 180.0},
    {"LEFT", true, 7, -6.0, 0.0, -90.0},
    {"RIGHT", true, 12, 6.0, 0.0, 90.0},
}};

// Field: origin at field center, inches (same as path.jerryio and our autons).
// Inside faces of the perimeter walls. TODO: measure wall-to-wall and halve it.
inline constexpr double WALL_X = 70.2; // walls at x = -WALL_X and x = +WALL_X
inline constexpr double WALL_Y = 70.2; // walls at y = -WALL_Y and y = +WALL_Y

// Toggle on the left wall (x = -WALL_X), centered at TOGGLE_CENTER_Y.
// Modeled as a box sticking out of the wall. TODO: measure width and how far it sticks out.
inline constexpr bool TOGGLE_IN_MAP = false; // sensors sit below the toggle, so they see the wall behind it
inline constexpr double TOGGLE_CENTER_Y = 0.0;
inline constexpr double TOGGLE_WIDTH = 6.0; // along the wall (y), inches
inline constexpr double TOGGLE_DEPTH = 2.0; // out from the wall face (x), inches

// ===================== DEFAULTS (don't touch) =====================
inline constexpr int MIN_VALID_MM = 20;          // V5 distance sensor spec minimum
inline constexpr int MAX_VALID_MM = 2000;        // V5 spec maximum; readings get noisy past this
inline constexpr int CONFIDENCE_MIN_MM = 200;    // sensor only reports confidence beyond 200 mm
inline constexpr int MIN_CONFIDENCE = 30;        // 0-63 scale; below this the reading is a guess
inline constexpr double REF_DISTANCE_IN = 42.0;  // distance R is measured at; R_eff = R * (d / 42)^2 since the
                                                 //   sensor's error grows roughly in proportion to distance
inline constexpr double MIN_R_SCALE = 0.1;       // floor on (d / 42)^2 so readings right at a wall (~13" or
                                                 //   closer) don't get an unrealistically tiny R
inline constexpr double MIN_R_BIAS_IN = 0.3;     // floor on R_BIAS_IN
inline constexpr std::uint32_t NOISE_SAMPLE_MS = 60; // R window takes at most one sample per sensor per
                                                 //   60 ms (> the ~33 ms sensor refresh), so every stored
                                                 //   sample is a fresh reading, never a repeat
inline constexpr double INITIAL_STD_IN = 1.0;    // starting position uncertainty after setPose
inline constexpr double MAX_CORRECTION_IN = 0.25; // max pose shift written back per cycle (12.5 in/s at 20 ms)
inline constexpr double TELEPORT_IN = 6.0;       // odom jump in one cycle bigger than this = someone called
                                                 //   setPose; reseed instead of treating it as motion
inline constexpr double MIN_INNOVATION_VAR = 1e-6; // guard against dividing by ~0 in the update
inline constexpr std::uint32_t LOOP_MS = 20;     // EKF task period
inline constexpr std::uint32_t SENSOR_PERIOD_MS = 33; // V5 distance sensor refresh; don't reuse a sample
inline constexpr bool TERMINAL_TELEMETRY = true;  // CSV debug lines to the laptop terminal (USB / wireless)
inline constexpr std::uint32_t CSV_PERIOD_MS = 100; // terminal CSV rate
inline constexpr bool CONTROLLER_TELEMETRY = true; // EKF summary on the controller screen
inline constexpr std::uint32_t CONTROLLER_LINE_MS = 50; // controller drops commands sent faster than this;
                                                 //   one line is updated per period, rotating
// The EKF telemetry is the only code that writes to the controller screen.

static_assert(Q >= 0 && R > 0 && MAX_RESIDUAL > 0 && MAX_ANGLE_DEG > 0 && MAX_ANGLE_DEG < 90);
static_assert(WALL_X > 0 && WALL_Y > 0);
static_assert(REF_DISTANCE_IN > 0 && MIN_R_SCALE > 0);
static_assert(IMU_SCALE > 0.9 && IMU_SCALE < 1.1, "IMU_SCALE should be within 10% of 1.0; re-measure");

} // namespace ekf_config
