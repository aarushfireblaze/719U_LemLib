#pragma once

// Tuning output (EKF + IMU). Nothing in this project prints to the brain screen.
//  - Laptop terminal: CSV line every CSV_PERIOD_MS (TERMINAL_TELEMETRY).
//  - Controller: 3 short lines, one line updated per CONTROLLER_LINE_MS (CONTROLLER_TELEMETRY).
// Flags live in the DEFAULTS section of ekf_config.hpp.

namespace loc {
// Starts the telemetry task. Call once from initialize(). Runs even with EKF_ENABLED false so the
// controller still shows odometry X/Y, heading and IMU calibration failures (needed for tuning).
// imuOk = false shows "IMU CAL FAILED!" on the controller.
void startEkfTelemetry(bool imuOk);
} // namespace loc
