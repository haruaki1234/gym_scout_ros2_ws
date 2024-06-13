/**
 * @file numeric.hpp
 * @author Takuma Nakao
 * @brief 数値関連ヘルパー関数
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <cmath>
#include <Eigen/Core>

namespace tlab
{

/**
 * @brief 行列がInfを含むかどうか
 *
 * @param v 行列
 * @retval true 含む
 * @retval false 含まない
 */
static inline bool hasInf(const Eigen::MatrixXd& v) { return v.array().isInf().any(); }

/**
 * @brief 行列がNaNを含むかどうか
 *
 * @param v 行列
 * @retval true 含む
 * @retval false 含まない
 */
static inline bool hasNan(const Eigen::MatrixXd& v) { return v.array().isNaN().any(); }

} // namespace tlab