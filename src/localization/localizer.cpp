#include "localization/localizer.hpp"
#include <mutex>
#include "drivetrain.hpp"
#include "localization/ekf_config.hpp"

namespace loc {

namespace {
using namespace ekf_config;

// One above LemLib's odometry task (default priority). While this task runs without blocking,
// LemLib's odom task cannot preempt it, so "re-read pose, add correction, setPose" is atomic
// with respect to odometry updates. (PROS 4 does not expose scheduler suspend.)
constexpr std::uint32_t TASK_PRIORITY = TASK_PRIORITY_DEFAULT + 1;

Vec2 xy(const lemlib::Pose& p) { return {p.x, p.y}; }

Vec2 clampLength(Vec2 v, double maxLen) {
    const double len = norm(v);
    return len > maxLen ? (maxLen / len) * v : v;
}

std::array<DistanceSensorModel, 4> makeModels() {
    return {DistanceSensorModel(SENSORS[0]), DistanceSensorModel(SENSORS[1]),
            DistanceSensorModel(SENSORS[2]), DistanceSensorModel(SENSORS[3])};
}
} // namespace

Localizer localizer;

Localizer::Localizer() : map_(FieldMap::fromConfig()), models_(makeModels()) {}

void Localizer::start(lemlib::Pose initialPose) {
    if (!EKF_ENABLED) return;
    resetPose(initialPose);
    std::lock_guard<pros::Mutex> lock(mutex_); // check-and-create under the lock: never two tasks
    if (task_) return;
    task_ = new pros::Task(
        [this] {
            std::uint32_t wake = pros::millis();
            while (true) {
                step();
                pros::Task::delay_until(&wake, LOOP_MS);
            }
        },
        TASK_PRIORITY, TASK_STACK_DEPTH_DEFAULT, "EKF localizer");
    status_.running = true;
}

void Localizer::resetPose(lemlib::Pose pose) {
    std::lock_guard<pros::Mutex> lock(mutex_);
    chassis.setPose(pose);
    reseed(chassis.getPose(true));
}

void Localizer::reseed(lemlib::Pose pose) {
    lastOdom_ = pose;
    ekf_.reset(xy(pose), INITIAL_STD_IN * INITIAL_STD_IN);
}

void Localizer::setEnabled(bool enabled) { enabled_ = enabled; }

LocalizerStatus Localizer::getStatus() {
    std::lock_guard<pros::Mutex> lock(mutex_);
    return status_;
}

void Localizer::publish(const std::array<SensorResult, 4>& sensors) {
    status_.enabled = enabled_;
    status_.ekf = ekf_.state();
    status_.odom = xy(lastOdom_);
    status_.P = ekf_.covariance();
    status_.sensors = sensors;
    for (std::size_t i = 0; i < noise_.size(); ++i) {
        const NoiseMeter& m = noise_[i];
        status_.noise[i] = {m.count(), m.full(), m.mean(), m.stdDev(), NoiseMeter::bias(), m.estimatedR()};
    }
}

void Localizer::step() {
    std::lock_guard<pros::Mutex> lock(mutex_);
    std::array<SensorResult, 4> results = status_.sensors;

    // ---- Predict: LemLib's x/y change since last cycle is the motion model.
    const lemlib::Pose odom = chassis.getPose(true); // radians, LemLib heading convention
    const Vec2 delta = xy(odom) - xy(lastOdom_);
    const double traveled = norm(delta);
    if (traveled > TELEPORT_IN) { // someone called chassis.setPose without resetPose
        reseed(odom);
        publish(results);
        return;
    }
    ekf_.predict(delta, Q * traveled);
    lastOdom_ = odom;

    // Disabled: follow odometry (so re-enabling starts with no correction) but still evaluate the
    // sensors below, so measured vs. expected can be checked in telemetry without correcting.
    const bool correcting = enabled_;
    if (!correcting) ekf_.setState(xy(odom));

    // ---- Update: one scalar update per fresh, valid sensor. Expected distance is recomputed
    // from the latest EKF state each time (sequential updates).
    const std::uint32_t now = pros::millis();
    for (std::size_t i = 0; i < models_.size(); ++i) {
        const DistanceSensorModel& model = models_[i];
        if (!model.setup().enabled) {
            results[i] = SensorResult{};
            continue;
        }
        // PROS has no "new sample" flag; the sensor refreshes every ~33 ms, so don't reuse
        // a sample by reading faster than that. Keep showing the last result meanwhile.
        if (now - lastSampleMs_[i] < SENSOR_PERIOD_MS) continue;
        lastSampleMs_[i] = now;

        results[i] = model.evaluate(model.read(), ekf_.state(), odom.theta, map_);
        // Noise meter takes every real reading, even ones the EKF rejects for residual/angle:
        // during the R test the pose may be off, but the sensor's wobble is still valid.
        // Only fresh samples: PROS has no "new sample" flag, so space them further apart than the
        // sensor's refresh. A repeated reading would make the std dev look smaller than it is.
        const Reject why = results[i].reason;
        const bool realReading = why != Reject::NoObject && why != Reject::OutOfRange && why != Reject::LowConfidence;
        if (realReading && now - lastNoiseMs_[i] >= NOISE_SAMPLE_MS) {
            lastNoiseMs_[i] = now;
            noise_[i].add(results[i].measuredIn);
        }
        if (correcting && results[i].accepted()) {
            ekf_.update(results[i].measuredIn, results[i].expectedIn, results[i].H, results[i].r);
        }
    }

    // ---- Write back: shift LemLib's latest pose by (EKF - odom we predicted from), capped,
    // so any odometry LemLib integrated since our read is kept. Heading untouched.
    const Vec2 correction = clampLength(ekf_.state() - xy(odom), MAX_CORRECTION_IN);
    if (correcting && norm(correction) > 1e-4) {
        // No blocking calls between these two lines (see TASK_PRIORITY).
        const lemlib::Pose latest = chassis.getPose(true);
        chassis.setPose(latest.x + correction.x, latest.y + correction.y, latest.theta, true);
        // Count the correction as already applied so the next predict doesn't add it again.
        lastOdom_.x += correction.x;
        lastOdom_.y += correction.y;
    }
    publish(results);
}

} // namespace loc
