/**
 * @file transform.hpp
 * @author Takuma Nakao
 * @brief 2次元座標系における位置姿勢
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "./vector2.hpp"

namespace tlab
{

struct Transform {
    double x, y, theta;
    Transform() = default;
    constexpr Transform(const double _x, const double _y, const double _theta) : x(_x), y(_y), theta(_theta) {}
    constexpr Transform(const Vector2& _xy, const double _theta) : x(_xy.x), y(_xy.y), theta(_theta) {}
    constexpr Transform(const Transform&) = default;
    constexpr Vector2 make_vector2() const { return Vector2(x, y); }
    constexpr void set(double vx, double vy, double vtheta)
    {
        x = vx;
        y = vy;
        theta = vtheta;
    }
    double distance() const { return Vector2(x, y).norm(); }
    void rotate(const double theta) { rotate(Vector2::zero(), theta); }
    void rotate(const double rotX, const double rotY, const double theta) { rotate(Vector2(rotX, rotY), theta); }
    void rotate(Vector2 rotPos, const double theta)
    {
        Vector2 p = make_vector2() - rotPos;
        p.rotate(theta);
        x = p.x + rotPos.x;
        y = p.y + rotPos.y;
    }
    void set_polar(const double radius, const double angle, const double robotTheta)
    {
        x = radius * std::cos(angle);
        y = radius * std::sin(angle);
        theta = robotTheta;
    }
    bool is_zero() const { return approx_zero(x) && approx_zero(y) && approx_zero(theta); }
    bool is_zero_pos() const { return x == 0.0 && y == 0.0; }
    bool is_zero_angle() const { return theta == 0.0; }
    bool has_nan() const { return std::isnan(x) || std::isnan(y) || std::isnan(theta); }
    static constexpr double get_angle(Transform& a, Transform& b) { return b.theta - a.theta; };
    static double get_distance(Transform& a, Transform& b) { return (b - a).distance(); }
    static Transform get_lerp(Transform& a, Transform& b, const double t) { return {lerp(a.x, b.x, t), lerp(a.y, b.y, t), lerp(a.theta, b.theta, t)}; }
    static constexpr Transform origin() { return {0.0, 0.0, 0.0}; }
    constexpr bool operator==(const Transform v) const { return (x == v.x) && (y == v.y) && (theta == v.theta); }
    constexpr bool operator!=(const Transform& v) const { return !((x == v.x) && (y == v.y) && (theta == v.theta)); }
    constexpr Transform operator+(const Transform& v) const { return {x + v.x, y + v.y, theta + v.theta}; }
    constexpr Transform operator-(const Transform& v) const { return {x - v.x, y - v.y, theta - v.theta}; }
    constexpr Transform& operator+=(const Transform& v)
    {
        x += v.x;
        y += v.y;
        theta += v.theta;

        return *this;
    }
    constexpr Transform& operator-=(const Transform& v)
    {
        x -= v.x;
        y -= v.y;
        theta -= v.theta;

        return *this;
    }
    constexpr Transform operator+(const Vector2& v) const { return {x + v.x, y + v.y, theta}; }
    constexpr Transform operator-(const Vector2& v) const { return {x - v.x, y - v.y, theta}; }
    constexpr Transform operator+(const double angle) const { return {x, y, theta + angle}; }
    constexpr Transform operator-(const double angle) const { return {x, y, theta - angle}; }
    constexpr Transform& operator+=(const Vector2& v)
    {
        x += v.x;
        y += v.y;

        return *this;
    }
    constexpr Transform& operator-=(const Vector2& v)
    {
        x -= v.x;
        y -= v.y;

        return *this;
    }
    constexpr Transform& operator+=(const double angle)
    {
        theta += angle;

        return *this;
    }
    constexpr Transform& operator-=(const double angle)
    {
        theta -= angle;

        return *this;
    }
    constexpr Transform operator*(const double value) const { return {x * value, y * value, theta * value}; }
    constexpr Transform operator/(const double value) const { return {x / value, y / value, theta / value}; }
    constexpr Transform& operator*=(const double value)
    {
        x *= value;
        y *= value;
        theta *= value;

        return *this;
    }
    constexpr Transform& operator/=(const double value)
    {
        x /= value;
        y /= value;
        theta /= value;

        return *this;
    }
    double& operator[](const int index)
    {
        assert(0 <= index && index < 3);
        if (index == 0)
            return x;
        if (index == 1)
            return y;
        return theta;
    }
    constexpr double operator[](const int index) const
    {
        assert(0 <= index && index < 3);
        if (index == 0)
            return x;
        if (index == 1)
            return y;
        return theta;
    }
};

} // namespace tlab