#pragma once

#include <cmath>
#include <Eigen/Core>

namespace tlab
{

static inline bool hasInf(const Eigen::MatrixXd& v) { return v.array().isInf().any(); }

static inline bool hasNan(const Eigen::MatrixXd& v) { return v.array().isNaN().any(); }

} // namespace tlab