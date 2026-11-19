#pragma once
#include <functional>
#include <cassert>
#include <cstdint>
namespace pros {
struct TickFinished {};
inline std::function<void()> worker;
inline std::uint32_t now = 1000; // fake clock, ms
inline std::uint32_t millis() { return now; }
class Task {
public:
    explicit Task(std::function<void()> body) { worker = body; }
};
inline void delay(int milliseconds) { assert(milliseconds == 5); throw TickFinished{}; }
}
