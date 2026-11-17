#pragma once

#include "main.h"

// Claw piston solenoid on 3-wire port A. false = retracted, true = extended.
inline pros::adi::DigitalOut claw('A', false);
// Last value sent to the piston; shared by autonomous and driver control so Y always flips it.
inline bool clawExtended = false;

inline void setClaw(bool extended) {
    clawExtended = extended;
    claw.set_value(extended);
}
// Assumes extended = open. If the claw does the opposite, swap true/false in these two.
inline void openClaw() { setClaw(true); }
inline void closeClaw() { setClaw(false); }
