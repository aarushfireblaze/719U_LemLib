// Off-robot tests for src/localization/ekf.cpp. Run: tests/run_ekf_tests.sh

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "localization/ekf.hpp"
#include "localization/noise_meter.hpp"

namespace {
int failures = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) < tol; }

void predictShiftsStateAndGrowsVariance() {
    loc::Ekf ekf;
    ekf.reset({1, 2}, 4);
    ekf.predict({3, -1}, 0.5);
    check(near(ekf.state().x, 4) && near(ekf.state().y, 1), "predict shifts state by delta");
    const auto P = ekf.covariance();
    check(near(P.a, 4.5) && near(P.d, 4.5) && near(P.b, 0) && near(P.c, 0), "predict adds q to x and y variance");
}

void scalarUpdateMatchesHandMath() {
    // P = 4, R = 1, H = (1, 0): K = 4/5. Innovation 5 moves x by 4. New var = (1-K)^2*4 + K^2*1 = 0.8.
    loc::Ekf ekf;
    ekf.reset({0, 0}, 4);
    check(ekf.update(15, 10, {1, 0}, 1), "update accepted");
    check(near(ekf.state().x, 4) && near(ekf.state().y, 0), "update moves only the observed axis");
    const auto P = ekf.covariance();
    check(near(P.a, 0.8) && near(P.d, 4), "Joseph-form covariance");
}

void obliqueUpdateStaysSymmetric() {
    loc::Ekf ekf;
    ekf.reset({0, 0}, 2);
    for (int i = 0; i < 1000; ++i) {
        ekf.update(1.0, 0.0, {0.6, -1.1}, 0.3);
        ekf.update(-1.0, 0.0, {-1.2, 0.2}, 0.3);
    }
    const auto P = ekf.covariance();
    check(P.b == P.c, "covariance exactly symmetric");
    check(P.a > 0 && P.d > 0 && P.a * P.d - P.b * P.c > 0, "covariance positive definite");
}

void tinyInnovationVarianceIsRejected() {
    loc::Ekf ekf;
    ekf.reset({1, 1}, 0);
    check(!ekf.update(5, 0, {1, 0}, 0), "S ~ 0 refused");
    check(near(ekf.state().x, 1), "refused update leaves state alone");
}

void noiseMeterEstimatesR() {
    using loc::NoiseMeter;
    const double bias = NoiseMeter::bias();
    check(bias >= 0.3, "bias floored at 0.3");

    NoiseMeter m;
    check(m.count() == 0 && !m.full() && near(m.stdDev(), 0), "empty meter");

    // 41 and 43: mean 42, sample std dev sqrt(2) (n - 1 = 1). R = (std + bias)^2 at 42".
    m.add(41);
    m.add(43);
    check(near(m.mean(), 42) && near(m.stdDev(), std::sqrt(2.0)), "true sample std dev");
    const double e = std::sqrt(2.0) + bias;
    check(near(m.estimatedR(), e * e), "R = (std + bias)^2 at reference distance");

    // Same wobble at 84" is 4x the distance scale, so R at 42" is 4x smaller.
    NoiseMeter far;
    far.add(83);
    far.add(85);
    check(near(far.estimatedR(), e * e / 4), "R converted to 42 in");

    // A perfectly steady sensor still gets the bias.
    NoiseMeter steady;
    for (std::size_t i = 0; i < NoiseMeter::WINDOW; ++i) steady.add(42.0);
    check(steady.full() && near(steady.estimatedR(), bias * bias), "zero noise -> R = bias^2");

    // Old readings roll out of the window.
    for (std::size_t i = 0; i < NoiseMeter::WINDOW; ++i) m.add(50.0);
    check(m.count() == NoiseMeter::WINDOW && near(m.stdDev(), 0) && near(m.mean(), 50), "window rolls over");
}
} // namespace

int main() {
    predictShiftsStateAndGrowsVariance();
    scalarUpdateMatchesHandMath();
    obliqueUpdateStaysSymmetric();
    tinyInnovationVarianceIsRejected();
    noiseMeterEstimatesR();
    if (failures) return EXIT_FAILURE;
    std::printf("ekf tests passed\n");
    return EXIT_SUCCESS;
}
