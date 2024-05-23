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

static inline bool hasInf(const Eigen::MatrixXd& v) { return v.array().isInf().any(); }

static inline bool hasNan(const Eigen::MatrixXd& v) { return v.array().isNaN().any(); }

} // namespace tlab