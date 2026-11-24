#include "localization/distance_sensor_model.hpp"
#include <algorithm>

namespace loc {

namespace {
constexpr double MM_PER_IN = 25.4;
constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

double orGlobal(double override, double global) { return override >= 0 ? override : global; }
} // namespace

double effectiveR(double r, double distanceIn) {
    // Sensor error grows roughly in proportion to distance, so its variance grows with distance^2.
    // Uses the measured distance: that's the range the sensor actually worked at.
    const double ratio = distanceIn / ekf_config::REF_DISTANCE_IN;
    return r * std::max(ratio * ratio, ekf_config::MIN_R_SCALE);
}

const char* rejectCode(Reject r) {
    switch (r) {
        case Reject::None: return "OK";
        case Reject::Disabled: return "OFF";
        case Reject::NoObject: return "NONE";
        case Reject::Stale: return "OLD";
        case Reject::OutOfRange: return "RNG";
        case Reject::LowConfidence: return "CONF";
        case Reject::BadAngle: return "ANG";
        case Reject::NoHit: return "MISS";
        case Reject::Residual: return "RES";
    }
    return "?";
}

DistanceSensorModel::DistanceSensorModel(const ekf_config::SensorSetup& setup)
    : setup_(setup),
      r_(orGlobal(setup.r, ekf_config::R)),
      maxResidual_(orGlobal(setup.maxResidual, ekf_config::MAX_RESIDUAL)),
      cosMaxAngle_(std::cos(orGlobal(setup.maxAngleDeg, ekf_config::MAX_ANGLE_DEG) * DEG_TO_RAD)) {}

SensorResult DistanceSensorModel::predict(Vec2 robotXY, double headingRad, const FieldMap& map) const {
    SensorResult out;
    out.r = r_;
    out.measuredIn = -1;
    out.expectedIn = -1;

    // LemLib heading: 0 = +Y, clockwise positive. So the robot's forward unit vector is
    // (sin h, cos h) and its right unit vector is (cos h, -sin h).
    const double s = std::sin(headingRad), c = std::cos(headingRad);
    const Vec2 forward{s, c}, right{c, -s};
    const Vec2 origin = robotXY + setup_.offsetX * right + setup_.offsetY * forward;

    const double beam = headingRad + setup_.mountAngleDeg * DEG_TO_RAD;
    const Vec2 dir{std::sin(beam), std::cos(beam)};

    const auto hit = map.raycast(origin, dir);
    if (!hit) {
        out.reason = Reject::NoHit;
        return out;
    }
    out.expectedIn = hit->distance;

    // Jacobian. The wall is the line n.q = c (n = unit normal). The ray hits it at
    //   t = (c - n.p) / (n.d)
    // where p = sensor position, d = beam direction. Heading is held fixed, so p moves 1:1
    // with the robot's (x, y), giving
    //   H = dt/d(x, y) = -n / (n.d).
    // |n.d| = cos(incidence angle), so moving 1 inch straight toward the wall shortens the beam
    // by 1/cos(incidence). The formula gives the same H for n and -n, so the normal's sign is irrelevant.
    const double nd = dot(hit->normal, dir);
    const double cosIncidence = std::fabs(nd);
    out.H = (-1.0 / nd) * hit->normal;
    out.reason = cosIncidence < cosMaxAngle_ ? Reject::BadAngle : Reject::None;
    return out;
}

SensorResult DistanceSensorModel::evaluate(RawReading raw, Vec2 robotXY, double headingRad,
                                           const FieldMap& map) const {
    using namespace ekf_config;
    SensorResult out;
    out.r = r_;
    out.measuredIn = -1;
    out.expectedIn = -1;
    if (!setup_.enabled) return out; // reason defaults to Disabled

    // Expected distance first so telemetry shows it even when the reading is rejected.
    out = predict(robotXY, headingRad, map);

    if (raw.mm <= 0 || raw.mm >= 9999) {
        out.reason = Reject::NoObject;
        return out;
    }
    out.measuredIn = raw.mm / MM_PER_IN;
    out.r = effectiveR(r_, out.measuredIn);
    if (raw.mm < MIN_VALID_MM || raw.mm > MAX_VALID_MM) {
        out.reason = Reject::OutOfRange;
        return out;
    }
    if (raw.mm > CONFIDENCE_MIN_MM && raw.confidence < MIN_CONFIDENCE) {
        out.reason = Reject::LowConfidence;
        return out;
    }
    if (out.reason != Reject::None) return out; // NoHit or BadAngle from predict()
    if (std::fabs(out.measuredIn - out.expectedIn) > maxResidual_) out.reason = Reject::Residual;
    return out;
}

} // namespace loc
