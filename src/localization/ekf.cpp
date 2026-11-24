#include "localization/ekf.hpp"
#include "localization/ekf_config.hpp"

namespace loc {

void Ekf::reset(Vec2 xy, double variance) {
    x_ = xy;
    P_ = {variance, 0, 0, variance};
}

void Ekf::predict(Vec2 delta, double q) {
    // The motion model is "LemLib's odometry delta is right", so F = I and the state just shifts.
    // Odometry error grows with distance driven, so q is scaled by distance by the caller.
    x_ = x_ + delta;
    P_.a += q;
    P_.d += q;
}

bool Ekf::update(double z, double h, Vec2 H, double r) {
    // Innovation variance S = H P H^T + R (scalar).
    const Vec2 PHt = P_ * H; // P is symmetric, so P H^T == P^T H^T
    const double S = dot(H, PHt) + r;
    if (S < ekf_config::MIN_INNOVATION_VAR) return false;

    const Vec2 K = (1.0 / S) * PHt; // Kalman gain (2x1)
    x_ = x_ + (z - h) * K;

    // Joseph form: P = (I - K H) P (I - K H)^T + K R K^T. Stays symmetric positive
    // semi-definite even with rounding, unlike the short form P = (I - K H) P.
    const Mat2 IKH = identity() - outer(K, H);
    Mat2 P = IKH * P_ * transpose(IKH) + outer(K, r * K);
    // Symmetrize to remove any leftover rounding asymmetry.
    const double off = 0.5 * (P.b + P.c);
    P.b = P.c = off;
    P_ = P;
    return true;
}

} // namespace loc
