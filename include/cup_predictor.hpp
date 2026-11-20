#pragma once

#include <cstdint>

// Tracks a cup approaching the distance sensor and predicts where it will be shortly.
// Model: constant acceleration, d(t) = d + v*t + a*t^2/2, with v and a smoothed (EMA)
// because the sensor is only ~30 Hz and noisy (~±15 mm up close).
// Units: mm, ms. Negative velocity = cup getting closer.
struct CupPredictor {
    float lookaheadMs;      // how far ahead to predict (sensor lag + piston close time)
    float smoothing = 0.5f; // weight of the newest sample, 0..1

    float distance = 0, velocity = 0, accel = 0;
    std::uint32_t lastMs = 0;
    bool primed = false;

    // Feed one NEW sensor reading (call only when the value changes).
    void update(float mm, std::uint32_t nowMs) {
        if (primed && nowMs > lastMs) {
            const float dt = static_cast<float>(nowMs - lastMs);
            const float prevVelocity = velocity;
            velocity += smoothing * ((mm - distance) / dt - velocity);
            accel += smoothing * ((velocity - prevVelocity) / dt - accel);
        }
        distance = mm;
        lastMs = nowMs;
        primed = true;
    }

    // Cup is holding still (no new readings): drop motion estimate.
    void stop() { velocity = accel = 0; }

    // Nothing in view: start over.
    void reset() { primed = false; stop(); }

    float predicted() const {
        const float t = lookaheadMs;
        return distance + velocity * t + 0.5f * accel * t * t;
    }

    // In range now, or approaching and predicted to be in range within lookaheadMs.
    bool willBeWithin(float rangeMm) const {
        return primed && (distance <= rangeMm || (velocity < 0 && predicted() <= rangeMm));
    }
};
