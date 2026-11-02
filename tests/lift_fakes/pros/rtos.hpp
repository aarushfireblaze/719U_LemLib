#pragma once
#include <functional>
#include <cassert>
namespace pros {
struct TickFinished {};
inline std::function<void()> worker;
class Task {
public:
    explicit Task(std::function<void()> body) { worker = body; }
};
inline void delay(int milliseconds) { assert(milliseconds == 5); throw TickFinished{}; }
}
