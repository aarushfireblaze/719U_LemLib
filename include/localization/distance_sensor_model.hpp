#pragma once

// One instance per physical distance sensor: reading, validity checks, expected distance and Jacobian.
// Everything except read() is pure math (tested off-robot). read() lives in distance_sensor_io.cpp,
// the only file here that talks to PROS.

#include <cstdint>
#include "localization/ekf_config.hpp"
#include "localization/field_map.hpp"

namespace loc {

enum class Reject : std::uint8_t {
    None,          // accepted
    Disabled,      // sensor's enabled flag is false
    NoObject,      // sensor saw nothing / unplugged
    Stale,         // no new sample since last use
    OutOfRange,    // outside the sensor's valid range
    LowConfidence, // sensor not sure of its own reading
    BadAngle,      // beam hits the wall too far from perpendicular
    NoHit,         // beam misses every wall in the map
    Residual,      // measured vs expected too different (beam blocked)
};

const char* rejectCode(Reject r); // short code for the screen, e.g. "OK", "RES"

// Measurement noise at this distance: r * (distanceIn / REF_DISTANCE_IN)^2, floored at r * MIN_R_SCALE.
double effectiveR(double r, double distanceIn);

struct RawReading {
    int mm;          // raw distance in mm; <= 0 or >= 9999 means nothing seen
    int confidence;  // 0-63
};

struct SensorResult {
    Reject reason = Reject::Disabled;
    double measuredIn = 0; // -1 when there is no usable reading
    double expectedIn = 0; // -1 when the ray hit nothing
    Vec2 H;                // d(expected)/d(x, y)
    double r = 0;          // measurement noise to use (already scaled for distance)
    bool accepted() const { return reason == Reject::None; }
};

class DistanceSensorModel {
public:
    explicit DistanceSensorModel(const ekf_config::SensorSetup& setup);

    const ekf_config::SensorSetup& setup() const { return setup_; }

    // PROS read (distance_sensor_io.cpp).
    RawReading read() const;

    // Expected reading at this robot pose. headingRad uses LemLib convention:
    // 0 = +Y, clockwise positive.
    SensorResult predict(Vec2 robotXY, double headingRad, const FieldMap& map) const;

    // predict() plus every rejection check against a real reading.
    SensorResult evaluate(RawReading raw, Vec2 robotXY, double headingRad, const FieldMap& map) const;

private:
    ekf_config::SensorSetup setup_;
    double r_, maxResidual_, cosMaxAngle_;
};

} // namespace loc
