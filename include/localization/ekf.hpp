#pragma once

// 2-state EKF on field (x, y). Pure math, no PROS, unit-tested off-robot.
// Heading is NOT estimated: it comes from the IMU through LemLib.

#include "localization/linalg.hpp"

namespace loc {

class Ekf {
public:
    void reset(Vec2 xy, double variance);
    void setState(Vec2 xy) { x_ = xy; }

    // Odometry moved the robot by delta. q = process noise added to both x and y variance.
    void predict(Vec2 delta, double q);

    // Scalar measurement z with expected value h and Jacobian H (1x2), noise variance r.
    // Returns false (no change) if the innovation variance is too small to divide by.
    bool update(double z, double h, Vec2 H, double r);

    Vec2 state() const { return x_; }
    Mat2 covariance() const { return P_; }

private:
    Vec2 x_;
    Mat2 P_;
};

} // namespace loc
