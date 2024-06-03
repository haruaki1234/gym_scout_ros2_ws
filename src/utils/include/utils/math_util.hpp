/**
 * @file math_util.hpp
 * @author Takuma Nakao
 * @brief 数学関連ヘルパー関数
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
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

//! 円周率
constexpr double PI = 3.1415926535897932384626433832795;
//! 円周率の半分
constexpr double HALF_PI = PI / 2.0;
//! 円周率の2倍
constexpr double TWO_PI = PI * 2.0;
弧度法からラジアン法への変換係数
constexpr double DEG_TO_RAD = PI / 180.0;
//! ラジアン法から弧度法への変換係数
constexpr double RAD_TO_DEG = 180.0 / PI;
//! ネイピア数
constexpr double EULER = 2.718281828459045235360287471352;
//! 重力加速度
constexpr double GRAVITY = 9.807;
//! Nmからgfmへの変換係数
constexpr double Nm2gfm = (1 / GRAVITY);
//! gfmからNmへの変換係数
constexpr double gfm2Nm = GRAVITY;
//! mNmからgfcmへの変換係数
constexpr double mNm2gfcm = (Nm2gfm * 100);
//! gfcmからmNmへの変換係数
constexpr double gfcm2mNm = (gfm2Nm / 100);

} // namespace constants

/**
 * @brief 2つの値の間にあるかどうかを判定
 *
 * @tparam T 値型
 * @param x 判定したい値
 * @param min 最小値
 * @param max 最大値
 * @retval true 間にある
 * @retval false 間にない
 */
template<typename T>
static constexpr bool in_range_open(T x, T min, T max)
{
    return ((min < x && x < max) ? true : false);
}

/**
 * @brief 2つの値と等しいかその間にあるかどうかを判定
 *
 * @tparam T 値型
 * @param x 判定したい値
 * @param min 最小値
 * @param max 最大値
 * @retval true 間にある
 * @retval false 間にない
 */
template<typename T>
static constexpr bool in_range(T x, T min, T max)
{
    return ((min <= x && x <= max) ? true : false);
}

/**
 * @brief 符号を返す
 *
 * @tparam T 値型
 * @param x 符号を取得したい値
 * @return constexpr int 符号
 */
template<typename T>
static constexpr int sgn(T x)
{
    return (x > 0 ? 1 : x < 0 ? -1 : 0);
}

/**
 * @brief 弧度法からラジアン法へ変換
 *
 * @tparam T 値型
 * @param deg 弧度法
 * @return constexpr double ラジアン法
 */
template<typename T>
static constexpr double radians(T deg)
{
    return (deg * DEG_TO_RAD);
}

/**
 * @brief ラジアン法から弧度法へ変換
 *
 * @tparam T 値型
 * @param rad ラジアン法
 * @return constexpr double 弧度法
 */
template<typename T>
static constexpr double degrees(T rad)
{
    return (rad * RAD_TO_DEG);
}

/**
 * @brief 角度を0~2piに正規化
 *
 * @param angle 角度
 * @return double 正規化された角度
 */
static inline double normalize_angle_positive(double angle) { return std::fmod(std::fmod(angle, TWO_PI) + TWO_PI, TWO_PI); }

/**
 * @brief 角度を-pi~piに正規化
 *
 * @param angle 角度
 * @return double 正規化された角度
 */
static inline double normalize_angle(double angle)
{
    double a = normalize_angle_positive(angle);
    if (a > PI)
        a -= TWO_PI;
    return a;
}

/**
 * @brief 2つの角度の最小の差を計算
 *
 * @param from 開始角度
 * @param to 終了角度
 * @return double 角度差
 */
static inline double shortest_angular_distance(double from, double to) { return normalize_angle(to - from); }

/**
 * @brief 2乗を計算
 *
 * @param x 値
 * @return constexpr double 2乗された値
 */
constexpr inline double square(const double x) { return x * x; }

/**
 * @brief 3乗を計算
 *
 * @param x 値
 * @return constexpr double 3乗された値
 */
constexpr inline double cubic(const double x) { return x * x * x; }

/**
 * @brief 線形補間を計算
 *
 * @param a 値1
 * @param b 値2
 * @param t 比率
 * @return constexpr double 補間された値
 */
constexpr inline double lerp(const double a, const double b, const double t) { return a + (b - a) * t; }

/**
 * @brief 2つの値がおおよそ等しいかどうかを判定
 *
 * @param a 値1
 * @param b 値2
 * @retval true おおよそ等しい
 * @retval false おおよそ等しくない
 */
constexpr inline bool approx_eq(const double a, const double b) { return (std::abs(a - b) < 1e-12); }

/**
 * @brief 値がおおよそ0かどうかを判定
 *
 * @param a 値
 * @retval true おおよそ0
 * @retval false おおよそ0でない
 */
constexpr inline bool approx_zero(const double a) { return (std::abs(a) < 1e-12); }

} // namespace tlab