/**
 * @file vector2.hpp
 * @author Takuma Nakao
 * @brief 2次元ベクトル
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "./math_util.hpp"

namespace tlab
{

struct Vector2 {
    double x, y;
    Vector2() = default;
    constexpr Vector2(double vx, double vy) : x(vx), y(vy) {}
    constexpr Vector2(const Vector2&) = default;
    constexpr void set(const double vx, const double vy)
    {
        x = vx;
        y = vy;
    }
    void set_polar(const double radius, const double theta)
    {
        x = radius * std::cos(theta);
        y = radius * std::sin(theta);
    }
    constexpr double dot(const Vector2& v) const { return (x * v.x + y * v.y); }
    constexpr double cross(const Vector2& v) const { return (x * v.y - y * v.x); }
    constexpr double norm_sq() const { return dot(*this); }
    constexpr double norm() const { return std::sqrt(norm_sq()); }
    void normalize() { *this /= norm(); }
    constexpr double angle() const { return std::atan2(y, x); }
    constexpr Vector2 get_normalized()
    {
        Vector2 v = *this;
        v /= v.norm();
        return v;
    }
    void rotate(const double theta)
    {
        Vector2 v = *this;
        x = v.x * std::cos(theta) - v.y * std::sin(theta);
        y = v.x * std::sin(theta) + v.y * std::cos(theta);
    }
    constexpr Vector2 get_rotated(const double theta) const
    {
        Vector2 v = {x * std::cos(theta) - y * std::sin(theta), x * std::sin(theta) + y * std::cos(theta)};

        return v;
    }
    constexpr bool is_zero() const { return approx_zero(x) && approx_zero(y); }
    constexpr bool has_nan() const { return std::isnan(x) || std::isnan(y); }
    constexpr Vector2 yx() const { return {y, x}; }
    constexpr Vector2 nyx() const { return {-y, x}; }
    constexpr Vector2 ynx() const { return {y, -x}; }
    constexpr Vector2 nxy() const { return {-x, y}; }
    constexpr Vector2 xny() const { return {x, -y}; }
    static constexpr double dot(const Vector2& a, const Vector2& b) { return a.dot(b); }
    static constexpr double angle(const Vector2& a, const Vector2& b)
    {
        double value = a.dot(b) / (a.norm() * b.norm());
        return std::acos(value);
    }
    static constexpr double distance(const Vector2& a, const Vector2& b) { return (b - a).norm(); }
    static constexpr Vector2 lerp(const Vector2& a, const Vector2& b, const double t) { return {tlab::lerp(a.x, b.x, t), tlab::lerp(a.y, b.y, t)}; }
    constexpr Vector2 lerp(const Vector2& b, const double t) const { return lerp(*this, b, t); }
    static constexpr Vector2 zero() { return {0, 0}; }
    static constexpr Vector2 up() { return {0, 1}; }
    static constexpr Vector2 down() { return {0, -1}; }
    static constexpr Vector2 right() { return {1, 0}; }
    static constexpr Vector2 left() { return {-1, 0}; }
    constexpr Vector2 operator+() const { return *this; }
    constexpr Vector2 operator-() const { return {-x, -y}; }
    constexpr bool operator==(const Vector2& v) const { return (x == v.x) && (y == v.y); }
    constexpr bool operator!=(const Vector2& v) const { return !((x == v.x) && (y == v.y)); }
    constexpr Vector2 operator+(const Vector2& v) const { return {x + v.x, y + v.y}; }
    constexpr Vector2 operator-(const Vector2& v) const { return {x - v.x, y - v.y}; }
    friend constexpr Vector2 operator/(const Vector2& v, const double value) { return {v.x / value, v.y / value}; }
    constexpr Vector2& operator+=(const Vector2& v)
    {
        x += v.x;
        y += v.y;

        return *this;
    }
    constexpr Vector2& operator-=(const Vector2& v)
    {
        x -= v.x;
        y -= v.y;

        return *this;
    }
    constexpr Vector2& operator*=(const double value)
    {
        x *= value;
        y *= value;

        return *this;
    }
    constexpr Vector2& operator/=(const double value)
    {
        x /= value;
        y /= value;

        return *this;
    }
    double& operator[](const size_t index)
    {
        assert(index < 2);
        if (index == 0)
            return x;
        return y;
    }
    constexpr double operator[](const size_t index) const
    {
        assert(index < 2);
        if (index == 0)
            return x;
        return y;
    }
    template<typename Scaler>
    friend constexpr Vector2 operator*(const Vector2& v, const Scaler s) noexcept
    {
        return {v.x * static_cast<double>(s), v.y * static_cast<double>(s)};
    }
    template<typename Scaler>
    friend constexpr Vector2 operator*(const Scaler s, const Vector2& v) noexcept
    {
        return (v * static_cast<double>(s));
    }
};

} // namespace tlab