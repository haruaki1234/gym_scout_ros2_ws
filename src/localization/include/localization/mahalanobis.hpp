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