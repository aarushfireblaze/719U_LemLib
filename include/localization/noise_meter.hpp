#pragma once

// Measures one distance sensor's noise for tuning R (EKF_TUNING.md step 5).
// Keeps the last WINDOW fresh readings and estimates
//   R at this distance = (std dev + bias)^2
// where std dev is the true sample standard deviation of the stored readings and bias is
// ekf_config::R_BIAS_IN (floored at MIN_R_BIAS_IN). It then converts to R at REF_DISTANCE_IN (42")
// with the same distance scaling the EKF uses, so the result goes straight into ekf_config::R.
// Pure math, no PROS. The caller is responsible for only adding fresh samples.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include "localization/ekf_config.hpp"

namespace loc {

class NoiseMeter {
public:
    static constexpr std::size_t WINDOW = 75; // at one sample per ~80 ms, about 6 s

    void add(double inches) {
        samples_[next_] = inches;
        next_ = (next_ + 1) % WINDOW;
        count_ = std::min(count_ + 1, WINDOW);
    }

    std::size_t count() const { return count_; }
    bool full() const { return count_ == WINDOW; }

    double mean() const {
        double sum = 0;
        for (std::size_t i = 0; i < count_; ++i) sum += samples_[i];
        return count_ ? sum / count_ : 0;
    }

    // Sample standard deviation (n - 1).
    double stdDev() const {
        if (count_ < 2) return 0;
        const double m = mean();
        double sq = 0;
        for (std::size_t i = 0; i < count_; ++i) sq += (samples_[i] - m) * (samples_[i] - m);
        return std::sqrt(sq / (count_ - 1));
    }

    static double bias() { return std::max(ekf_config::R_BIAS_IN, ekf_config::MIN_R_BIAS_IN); }

    // R to put in ekf_config (i.e. R at REF_DISTANCE_IN).
    double estimatedR() const {
        const double e = stdDev() + bias();
        const double ratio = mean() / ekf_config::REF_DISTANCE_IN;
        return e * e / std::max(ratio * ratio, ekf_config::MIN_R_SCALE);
    }

private:
    std::array<double, WINDOW> samples_{}; // until full, only the first count_ slots hold data
    std::size_t next_ = 0;
    std::size_t count_ = 0;
};

} // namespace loc
