// The only PROS-dependent part of DistanceSensorModel. Kept separate so the model's math
// compiles off-robot without PROS.

#include "localization/distance_sensor_model.hpp"
#include "pros/distance.h"

namespace loc {

RawReading DistanceSensorModel::read() const {
    // PROS_ERR (unplugged) is a large negative int32; treated as "nothing seen" by evaluate().
    const std::int32_t mm = pros::c::distance_get(setup_.port);
    const std::int32_t conf = pros::c::distance_get_confidence(setup_.port);
    return {mm < 0 ? -1 : static_cast<int>(mm), conf < 0 ? 0 : static_cast<int>(conf)};
}

} // namespace loc
