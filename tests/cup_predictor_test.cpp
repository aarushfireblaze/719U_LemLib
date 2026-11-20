#include <cassert>
#include "cup_predictor.hpp"

int main() {
    // Cup closing at 1 mm/ms, 33 ms sensor period: triggers before it is actually in range.
    CupPredictor approach{60};
    std::uint32_t t = 0;
    float d = 200;
    while (!approach.willBeWithin(51)) { approach.update(d, t); d -= 33; t += 33; }
    assert(approach.distance > 51);

    // Cup holding still outside range: never triggers.
    CupPredictor still{60};
    for (std::uint32_t i = 0; i < 10; ++i) still.update(i % 2 ? 80 : 78, i * 33); // sensor jitter
    assert(!still.willBeWithin(51));

    // Cup moving away: never triggers even though it started close.
    CupPredictor away{60};
    for (std::uint32_t i = 0; i < 5; ++i) away.update(60 + i * 30, i * 33);
    assert(!away.willBeWithin(51));
}
