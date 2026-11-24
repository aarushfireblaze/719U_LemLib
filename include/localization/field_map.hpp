#pragma once

// Field geometry as line segments, plus ray casting. Pure math, no PROS.

#include <optional>
#include <vector>
#include "localization/linalg.hpp"

namespace loc {

struct Segment {
    Vec2 a;
    Vec2 b;
};

struct RayHit {
    double distance; // inches from ray origin to the hit
    Vec2 normal;     // unit normal of the segment that was hit (sign arbitrary)
};

class FieldMap {
public:
    explicit FieldMap(std::vector<Segment> segments) : segments_(std::move(segments)) {}

    // Perimeter walls (and the toggle, if enabled) from ekf_config.hpp.
    static FieldMap fromConfig();

    // Closest segment hit by the ray origin + t * dir (dir must be unit length), t > 0.
    std::optional<RayHit> raycast(Vec2 origin, Vec2 dir) const;

private:
    std::vector<Segment> segments_;
};

} // namespace loc
