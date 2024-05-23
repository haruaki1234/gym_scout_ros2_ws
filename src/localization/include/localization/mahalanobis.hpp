/**
 * @file mahalanobis.hpp
 * @author Takuma Nakao
 * @brief マハラノビス距離
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

namespace tlab
{

static inline double squaredMahalanobis(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::MatrixXd& C)
{
    const Eigen::VectorXd d = x - y;
    return d.dot(C.inverse() * d);
}

static inline double mahalanobis(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::MatrixXd& C) { return std::sqrt(squaredMahalanobis(x, y, C)); }

} // namespace tlab