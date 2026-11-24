#pragma once

// Glue: runs the EKF in a PROS task on top of LemLib odometry and writes the corrected
// x/y back into LemLib so moveToPoint / moveToPose see it. Heading is never touched.

#include <array>
#include <atomic>
#include <cstdint>
#include "lemlib/api.hpp"
#include "localization/distance_sensor_model.hpp"
#include "localization/ekf.hpp"
#include "localization/field_map.hpp"
#include "localization/noise_meter.hpp"
#include "pros/rtos.hpp"

namespace loc {

// Per-sensor noise over the last ~5 s, for tuning R (see NoiseMeter).
struct NoiseSummary {
    std::size_t count = 0; // fresh readings in the window (R is only trustworthy once full)
    bool full = false;
    double mean = 0;   // average distance, inches
    double stdDev = 0; // true sample std dev, inches
    double bias = 0;   // R_BIAS_IN used, inches
    double r = 0;      // (stdDev + bias)^2 converted to 42", ready for ekf_config::R
};

struct LocalizerStatus {
    bool running = false;
    bool enabled = false;
    Vec2 ekf;  // EKF estimate
    Vec2 odom; // LemLib pose used for the last predict
    Mat2 P;    // EKF covariance
    std::array<SensorResult, 4> sensors{};
    std::array<NoiseSummary, 4> noise{};
};

class Localizer {
public:
    Localizer();

    // Sets the LemLib pose, seeds the EKF and starts the task. Safe to call more than once:
    // only one task is ever created, and later calls just act as resetPose.
    // Does nothing if ekf_config::EKF_ENABLED is false.
    void start(lemlib::Pose initialPose);

    // Same as chassis.setPose(pose) plus reseeding the EKF. Use this whenever you set the pose.
    void resetPose(lemlib::Pose pose);

    // false = stop correcting (pose is pure LemLib odometry). Sensors are still read and
    // shown in telemetry, so offsets and noise can be checked without the EKF moving the pose.
    void setEnabled(bool enabled);

    LocalizerStatus getStatus();

private:
    void step();
    void reseed(lemlib::Pose pose); // caller holds mutex_
    void publish(const std::array<SensorResult, 4>& sensors); // caller holds mutex_

    pros::Mutex mutex_;
    pros::Task* task_ = nullptr;
    std::atomic<bool> enabled_{true};

    FieldMap map_;
    std::array<DistanceSensorModel, 4> models_;
    std::array<std::uint32_t, 4> lastSampleMs_{};
    std::array<NoiseMeter, 4> noise_{};
    std::array<std::uint32_t, 4> lastNoiseMs_{};
    Ekf ekf_;
    lemlib::Pose lastOdom_{0, 0, 0};
    LocalizerStatus status_;
};

extern Localizer localizer;

} // namespace loc

using loc::localizer;
