#pragma once
#include <cmath>
#include <cassert>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279
#endif

namespace tlab
{

inline namespace constants
{

constexpr double PI = 3.1415926535897932384626433832795;
constexpr double HALF_PI = PI / 2.0;
constexpr double TWO_PI = PI * 2.0;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double EULER = 2.718281828459045235360287471352;
constexpr double GRAVITY = 9.807;
constexpr double Nm2gfm = (1 / GRAVITY);
constexpr double gfm2Nm = GRAVITY;
constexpr double mNm2gfcm = (Nm2gfm * 100);
constexpr double gfcm2mNm = (gfm2Nm / 100);

} // namespace constants

template<typename T>
static constexpr bool in_range_open(T x, T min, T max)
{
    return ((min < x && x < max) ? true : false);
}
template<typename T>
static constexpr bool in_range(T x, T min, T max)
{
    return ((min <= x && x <= max) ? true : false);
}
template<typename T>
static constexpr int sgn(T x)
{
    return (x > 0 ? 1 : x < 0 ? -1 : 0);
}
template<typename T>
static constexpr double radians(T deg)
{
    return (deg * DEG_TO_RAD);
}
template<typename T>
static constexpr double degrees(T rad)
{
    return (rad * RAD_TO_DEG);
}
static inline double normalize_angle_positive(double angle) { return std::fmod(std::fmod(angle, TWO_PI) + TWO_PI, PI); }
static inline double normalize_angle(double angle)
{
    double a = normalize_angle_positive(angle);
    if (a > PI)
        a -= TWO_PI;
    return a;
}
static inline double shortest_angular_distance(double from, double to) { return normalize_angle(to - from); }
constexpr inline double square(const double x) { return x * x; }
constexpr inline double cubic(const double x) { return x * x * x; }
constexpr inline double lerp(const double a, const double b, const double t) { return a + (b - a) * t; }
constexpr inline double approx_eq(const double a, const double b) { return (std::abs(a - b) < 1e-12); }
constexpr inline double approx_zero(const double a) { return (std::abs(a) < 1e-12); }

} // namespace tlab