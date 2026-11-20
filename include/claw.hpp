#pragma once

#include <atomic>
#include "main.h"
#include "robot_config.hpp"
#include "cup_predictor.hpp"

// Claw piston solenoid on 3-wire port A. false = retracted, true = extended.
inline pros::adi::DigitalOut claw('A', false);
// Last value sent to the piston; shared by autonomous, driver control and the auto-clamp task.
inline std::atomic<bool> clawExtended{false};

inline void setClaw(bool extended) {
    clawExtended = extended;
    claw.set_value(extended);
}
// Assumes extended = open. If the claw does the opposite, swap true/false in these two.
inline void openClaw() { setClaw(true); }
inline void closeClaw() { setClaw(false); }

// ---- Auto-clamp: distance sensor closes the claw when a cup comes within range ----
inline pros::Distance clawDistance(config::clawDistancePort);
inline std::atomic<bool> autoClampOn{false};

// Turn the auto-clamp on or off at any time. Off = the task ignores the sensor.
inline void setAutoClamp(bool on) { autoClampOn = on; }

// Starts the background task once (call from initialize()). It never blocks other code.
inline void startAutoClamp() {
    static pros::Task task([] {
        CupPredictor cup{config::autoClampLookaheadMs};
        // Armed only after no cup is in range, so after the claw opens on a cup it is
        // still touching (scoring), it won't grab that same cup again.
        bool armed = false;
        int lastMm = -1;
        std::uint32_t lastNewMs = 0;
        while (true) {
            const int mm = clawDistance.get_distance(); // 9999 = nothing seen, PROS_ERR = unplugged
            const std::uint32_t now = pros::millis();
            if (mm <= 0 || mm > 2000) { cup.reset(); lastMm = -1; } // nothing in view
            else if (mm != lastMm) { cup.update(mm, now); lastMm = mm; lastNewMs = now; } // new reading
            else if (now - lastNewMs > 100) cup.stop(); // no change for 3 readings: cup at rest

            if (!cup.willBeWithin(config::autoClampRangeMm)) armed = true;
            else if (autoClampOn && armed && clawExtended) { // claw open + cup arriving
                closeClaw();
                armed = false;
            }
            pros::delay(10); // poll faster than the sensor so new readings are seen quickly
        }
    });
}
