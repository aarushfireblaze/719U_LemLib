#pragma once

// pros::Imu with a scale correction, handed to LemLib in place of a plain pros::Imu.
// LemLib 0.5.6 odometry reads heading only through the virtual get_rotation() (and uses its change
// each cycle), so overriding it is enough for LemLib to see the corrected heading.

#include <cmath>
#include <cstdint>
#include "pros/imu.hpp"
#include "pros/rtos.hpp"

class ScaledImu : public pros::Imu {
public:
    ScaledImu(std::uint8_t port, double scale) : pros::Imu(port), scale_(scale) {}

    // Raw rotation (unbounded degrees, clockwise positive) times the scale.
    double get_rotation() const override {
        const double raw = pros::Imu::get_rotation();
        return std::isfinite(raw) ? raw * scale_ : raw; // pass errors (PROS_ERR_F) through untouched
    }

    // Unscaled, unwrapped rotation straight from the IMU (for measuring IMU_SCALE).
    double rawRotation() const { return pros::Imu::get_rotation(); }

    // Kept consistent with get_rotation() so nothing sees an unscaled heading.
    double get_heading() const override {
        const double rot = get_rotation();
        if (!std::isfinite(rot)) return pros::Imu::get_heading();
        const double h = std::fmod(rot, 360.0);
        return h < 0 ? h + 360.0 : h;
    }

    // Blocks until calibration is really done. LemLib's calibrate checks is_calibrating() only
    // 10 ms after starting, which can be before the IMU raises the flag. So: give the flag a moment
    // to appear, then wait for it to clear. Returns false on timeout.
    bool waitUntilCalibrated(std::uint32_t timeoutMs = 3000) const {
        const std::uint32_t start = pros::millis();
        while (!is_calibrating() && pros::millis() - start < 100) pros::delay(5);
        while (is_calibrating()) {
            if (pros::millis() - start > timeoutMs) return false;
            pros::delay(10);
        }
        return true;
    }

private:
    double scale_;
};
