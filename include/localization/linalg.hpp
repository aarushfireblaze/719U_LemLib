#pragma once

// Tiny 2D vector/matrix helpers for the EKF. Header only, no dependencies.

#include <cmath>

namespace loc {

struct Vec2 {
    double x = 0;
    double y = 0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(double s, Vec2 v) { return {s * v.x, s * v.y}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double norm(Vec2 v) { return std::hypot(v.x, v.y); }

// Row-major 2x2: [a b; c d]
struct Mat2 {
    double a = 0, b = 0;
    double c = 0, d = 0;
};

inline Mat2 operator+(Mat2 m, Mat2 n) { return {m.a + n.a, m.b + n.b, m.c + n.c, m.d + n.d}; }
inline Mat2 operator-(Mat2 m, Mat2 n) { return {m.a - n.a, m.b - n.b, m.c - n.c, m.d - n.d}; }
inline Mat2 operator*(Mat2 m, Mat2 n) {
    return {m.a * n.a + m.b * n.c, m.a * n.b + m.b * n.d,
            m.c * n.a + m.d * n.c, m.c * n.b + m.d * n.d};
}
inline Vec2 operator*(Mat2 m, Vec2 v) { return {m.a * v.x + m.b * v.y, m.c * v.x + m.d * v.y}; }
inline Mat2 transpose(Mat2 m) { return {m.a, m.c, m.b, m.d}; }
inline Mat2 outer(Vec2 u, Vec2 v) { return {u.x * v.x, u.x * v.y, u.y * v.x, u.y * v.y}; }
inline Mat2 identity() { return {1, 0, 0, 1}; }

} // namespace loc
