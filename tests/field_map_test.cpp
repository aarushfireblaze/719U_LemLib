// Off-robot tests for ray casting (field_map.cpp) and the distance sensor model.
// Uses the real ekf_config.hpp, so tests that depend on SETUP values compute from them.
// Run: tests/run_ekf_tests.sh

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "localization/distance_sensor_model.hpp"
#include "localization/ekf.hpp"
#include "localization/ekf_config.hpp"
#include "localization/field_map.hpp"

namespace {
using namespace ekf_config;
using loc::Vec2;

int failures = 0;
constexpr double DEG = 3.14159265358979323846 / 180.0;

void check(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}

bool near(double a, double b, double tol = 1e-6) { return std::fabs(a - b) < tol; }

loc::FieldMap wallsOnly() {
    const double X = WALL_X, Y = WALL_Y;
    return loc::FieldMap({{{-X, -Y}, {-X, Y}}, {{X, -Y}, {X, Y}}, {{-X, -Y}, {X, -Y}}, {{-X, Y}, {X, Y}}});
}

void raycastStraightAndDiagonal() {
    const auto map = wallsOnly();
    auto hit = map.raycast({0, 0}, {1, 0});
    check(hit && near(hit->distance, WALL_X), "center -> right wall");
    check(hit && near(std::fabs(hit->normal.x), 1), "right wall normal is +-x");
    hit = map.raycast({10, -5}, {0, -1});
    check(hit && near(hit->distance, WALL_Y - 5), "offset origin -> bottom wall");
    const double s = std::sqrt(0.5);
    hit = map.raycast({0, 0}, {s, s});
    check(hit && near(hit->distance, WALL_X * std::sqrt(2.0)), "diagonal into corner");
    check(!map.raycast({WALL_X + 10, 0}, {1, 0}), "outside, pointing away = no hit");
}

void raycastToggle() {
    if (!TOGGLE_IN_MAP) return;
    const auto map = loc::FieldMap::fromConfig();
    const Vec2 left{-1, 0};
    auto hit = map.raycast({-60, TOGGLE_CENTER_Y}, left);
    check(hit && near(hit->distance, WALL_X - TOGGLE_DEPTH - 60), "beam at toggle hits its face");
    hit = map.raycast({-60, TOGGLE_CENTER_Y + TOGGLE_WIDTH}, left);
    check(hit && near(hit->distance, WALL_X - 60), "beam beside toggle hits the wall");
}

loc::DistanceSensorModel model(double offX, double offY, double mountDeg) {
    return loc::DistanceSensorModel(SensorSetup{"T", true, 1, offX, offY, mountDeg});
}

void expectedDistanceAndJacobian() {
    const auto map = wallsOnly();
    // Facing +Y (heading 0), sensor 6 in forward: top wall is WALL_Y - 6 away.
    auto r = model(0, 6, 0).predict({0, 0}, 0, map);
    check(r.accepted() && near(r.expectedIn, WALL_Y - 6), "front sensor expected distance");
    check(near(r.H.x, 0) && near(r.H.y, -1), "moving toward wall shortens beam 1:1");

    // Facing +X (heading 90). Right-side sensor (offset x = 6, mount 90) looks at -Y.
    r = model(6, 0, 90).predict({0, 0}, 90 * DEG, map);
    check(r.accepted() && near(r.expectedIn, WALL_Y - 6), "right sensor rotates with robot");
    check(near(r.H.x, 0) && near(r.H.y, 1), "moving +y moves away from bottom wall");

    // 20 deg off perpendicular: moving 1 in toward the wall shortens the beam by 1/cos(20).
    r = model(0, 0, 0).predict({0, 0}, 20 * DEG, map);
    check(near(r.expectedIn, WALL_Y / std::cos(20 * DEG)), "oblique expected distance");
    check(near(r.H.y, -1 / std::cos(20 * DEG)), "oblique Jacobian = 1/cos(incidence)");

    // Numerical check of H against finite differences.
    const auto m = model(3, -2, 45);
    const double h0 = 0.7, eps = 1e-5;
    const Vec2 p{12, -30};
    const auto base = m.predict(p, h0, map);
    const double dx = (m.predict({p.x + eps, p.y}, h0, map).expectedIn - base.expectedIn) / eps;
    const double dy = (m.predict({p.x, p.y + eps}, h0, map).expectedIn - base.expectedIn) / eps;
    check(near(base.H.x, dx, 1e-3) && near(base.H.y, dy, 1e-3), "H matches finite differences");
}

void rejections() {
    const auto map = wallsOnly();
    const auto m = model(0, 0, 0);
    const double expectMm = WALL_Y * 25.4;
    const int good = static_cast<int>(expectMm);
    check(m.evaluate({good, 63}, {0, 0}, 0, map).accepted(), "good reading accepted");
    check(m.evaluate({9999, 0}, {0, 0}, 0, map).reason == loc::Reject::NoObject, "9999 = no object");
    check(m.evaluate({-1, 0}, {0, 0}, 0, map).reason == loc::Reject::NoObject, "error = no object");
    check(m.evaluate({MIN_VALID_MM - 5, 63}, {0, 0}, 0, map).reason == loc::Reject::OutOfRange, "too close");
    check(m.evaluate({good, MIN_CONFIDENCE - 1}, {0, 0}, 0, map).reason == loc::Reject::LowConfidence,
          "low confidence");
    const int blocked = static_cast<int>((WALL_Y - MAX_RESIDUAL - 1) * 25.4);
    check(m.evaluate({blocked, 63}, {0, 0}, 0, map).reason == loc::Reject::Residual, "blocked beam");
    const double steep = (MAX_ANGLE_DEG + 5) * DEG;
    const int steepMm = static_cast<int>(WALL_Y / std::cos(steep) * 25.4);
    if (steepMm <= MAX_VALID_MM)
        check(m.evaluate({steepMm, 63}, {0, 0}, steep, map).reason == loc::Reject::BadAngle, "steep angle");
    SensorSetup off{"T", false, 1, 0, 0, 0};
    check(loc::DistanceSensorModel(off).evaluate({good, 63}, {0, 0}, 0, map).reason == loc::Reject::Disabled,
          "disabled sensor");
}

void rGrowsWithDistance() {
    check(near(loc::effectiveR(2.0, REF_DISTANCE_IN), 2.0), "R unchanged at reference distance");
    check(near(loc::effectiveR(2.0, 2 * REF_DISTANCE_IN), 8.0), "double distance = 4x R");
    check(near(loc::effectiveR(2.0, 0.5), 2.0 * MIN_R_SCALE), "R floored near the wall");
    const auto map = wallsOnly();
    const int mm = static_cast<int>(WALL_Y * 25.4);
    const auto r = model(0, 0, 0).evaluate({mm, 63}, {0, 0}, 0, map);
    check(near(r.r, loc::effectiveR(R, r.measuredIn)), "evaluate uses distance-scaled R");
}

void closedLoopConverges() {
    // Robot really at (-40, 20) facing 10 deg (inside MAX_ANGLE_DEG); odometry thinks (-42, 22). Feed perfect readings.
    const auto map = loc::FieldMap::fromConfig();
    const Vec2 truth{-40, 20};
    const double heading = 10 * DEG;
    loc::Ekf ekf;
    ekf.reset({-42, 22}, INITIAL_STD_IN * INITIAL_STD_IN);
    for (int i = 0; i < 50; ++i) {
        ekf.predict({0, 0}, Q * 0.5);
        for (const auto& setup : SENSORS) {
            const loc::DistanceSensorModel m(setup);
            const double trueIn = m.predict(truth, heading, map).expectedIn;
            const auto r = m.evaluate({static_cast<int>(std::lround(trueIn * 25.4)), 63}, ekf.state(), heading, map);
            if (r.accepted()) ekf.update(r.measuredIn, r.expectedIn, r.H, r.r);
        }
    }
    check(near(ekf.state().x, truth.x, 0.1) && near(ekf.state().y, truth.y, 0.1), "EKF converges to truth");
}
} // namespace

int main() {
    raycastStraightAndDiagonal();
    raycastToggle();
    expectedDistanceAndJacobian();
    rejections();
    rGrowsWithDistance();
    closedLoopConverges();
    if (failures) return EXIT_FAILURE;
    std::printf("field map / sensor model tests passed\n");
    return EXIT_SUCCESS;
}
