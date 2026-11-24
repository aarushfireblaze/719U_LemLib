#include "localization/field_map.hpp"
#include "localization/ekf_config.hpp"

namespace loc {

FieldMap FieldMap::fromConfig() {
    using namespace ekf_config;
    const double X = WALL_X, Y = WALL_Y;
    std::vector<Segment> s = {
        {{-X, -Y}, {-X, Y}}, // left
        {{X, -Y}, {X, Y}},   // right
        {{-X, -Y}, {X, -Y}}, // bottom
        {{-X, Y}, {X, Y}},   // top
    };
    if (TOGGLE_IN_MAP) {
        // Box on the left wall: front face plus the two sides.
        const double front = -X + TOGGLE_DEPTH;
        const double lo = TOGGLE_CENTER_Y - TOGGLE_WIDTH / 2, hi = TOGGLE_CENTER_Y + TOGGLE_WIDTH / 2;
        s.push_back({{front, lo}, {front, hi}});
        s.push_back({{-X, lo}, {front, lo}});
        s.push_back({{-X, hi}, {front, hi}});
    }
    return FieldMap(std::move(s));
}

std::optional<RayHit> FieldMap::raycast(Vec2 origin, Vec2 dir) const {
    std::optional<RayHit> best;
    for (const Segment& seg : segments_) {
        // Solve origin + t*dir = seg.a + s*e  for t (ray length) and s in [0, 1] (along segment).
        const Vec2 e = seg.b - seg.a;
        const double denom = dir.x * e.y - dir.y * e.x; // cross(dir, e); 0 = parallel
        if (std::fabs(denom) < 1e-9) continue;
        const Vec2 w = seg.a - origin;
        const double t = (w.x * e.y - w.y * e.x) / denom;
        const double s = (w.x * dir.y - w.y * dir.x) / denom;
        if (t <= 0 || s < 0 || s > 1) continue;
        if (!best || t < best->distance) {
            const double len = norm(e);
            best = RayHit{t, {-e.y / len, e.x / len}};
        }
    }
    return best;
}

} // namespace loc
