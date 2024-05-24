/**
 * @file covariance.hpp
 * @author Takuma Nakao
 * @brief 共分散ヘルパー関数
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "utils/covariance_index.hpp"

#include "matrix_types.hpp"
#include "state_index.hpp"

namespace tlab
{

/**
 * @brief EKFの共分散行列からPoseの共分散に変換
 *
 * @param P EKFの共分散行列
 * @return std::array<double, 36> Poseの共分散
 */
static inline std::array<double, 36> ekfCovarianceToPoseMessageCovariance(const Matrix6d& P)
{
    std::array<double, 36> covariance;
    covariance.fill(0.);

    covariance[XYZRPY_COV_IDX::X_X] = P(IDX::X, IDX::X);
    covariance[XYZRPY_COV_IDX::X_Y] = P(IDX::X, IDX::Y);
    covariance[XYZRPY_COV_IDX::X_YAW] = P(IDX::X, IDX::YAW);
    covariance[XYZRPY_COV_IDX::Y_X] = P(IDX::Y, IDX::X);
    covariance[XYZRPY_COV_IDX::Y_Y] = P(IDX::Y, IDX::Y);
    covariance[XYZRPY_COV_IDX::Y_YAW] = P(IDX::Y, IDX::YAW);
    covariance[XYZRPY_COV_IDX::YAW_X] = P(IDX::YAW, IDX::X);
    covariance[XYZRPY_COV_IDX::YAW_Y] = P(IDX::YAW, IDX::Y);
    covariance[XYZRPY_COV_IDX::YAW_YAW] = P(IDX::YAW, IDX::YAW);

    return covariance;
}

/**
 * @brief EKFの共分散行列からTwistの共分散に変換
 *
 * @param P EKFの共分散行列
 * @return std::array<double, 36> Twistの共分散
 */
static inline std::array<double, 36> ekfCovarianceToTwistMessageCovariance(const Matrix6d& P)
{
    std::array<double, 36> covariance;
    covariance.fill(0.);

    covariance[XYZRPY_COV_IDX::X_X] = P(IDX::VX, IDX::VX);
    covariance[XYZRPY_COV_IDX::X_Y] = P(IDX::VX, IDX::VY);
    covariance[XYZRPY_COV_IDX::X_YAW] = P(IDX::VX, IDX::WZ);
    covariance[XYZRPY_COV_IDX::Y_X] = P(IDX::VY, IDX::VX);
    covariance[XYZRPY_COV_IDX::Y_Y] = P(IDX::VY, IDX::VY);
    covariance[XYZRPY_COV_IDX::Y_YAW] = P(IDX::VY, IDX::WZ);
    covariance[XYZRPY_COV_IDX::YAW_X] = P(IDX::WZ, IDX::VX);
    covariance[XYZRPY_COV_IDX::YAW_Y] = P(IDX::WZ, IDX::VY);
    covariance[XYZRPY_COV_IDX::YAW_YAW] = P(IDX::WZ, IDX::WZ);

    return covariance;
}

} // namespace tlab