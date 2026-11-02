#include "lift_feedback.hpp"
#include <cassert>
#include <limits>
int main() {
    using namespace lift_feedback;
    assert(valid(-100, -110));
    assert(valid(1800, 1810));
    assert(position(-100, -110) == -105);
    assert(position(1800, 1810) == 1805);
    assert(valid(100, 116));
    assert(mismatched(100, 116));
    assert(!mismatched(100, 110));
    assert(!valid(std::numeric_limits<double>::infinity(), 0));
    assert(!valid(0, std::numeric_limits<double>::quiet_NaN()));
}
