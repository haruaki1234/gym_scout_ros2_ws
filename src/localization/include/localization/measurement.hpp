/**
 * @file measurement.hpp
 * @author Takuma Nakao
 * @brief 行列代入のヘルパー関数
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <Eigen/Core>

#include "utils/covariance_index.hpp"

#include "state_index.hpp"

namespace tlab
{

/**
 * @brief 位置姿勢の計測行列
 *
 * @return Eigen::Matrix<double, 3, 6> 位置姿勢の計測行列
 */
static inline Eigen::Matrix<double, 3, 6> poseMeasurementMatrix()
{
    Eigen::Matrix<double, 3, 6> C = Eigen::Matrix<double, 3, 6>::Zero();
    C(0, IDX::X) = 1.0;   // for pos x
    C(1, IDX::Y) = 1.0;   // for pos y
    C(2, IDX::YAW) = 1.0; // for yaw
    return C;
}

/**
 * @brief 速度の計測行列
 *
 * @return Eigen::Matrix<double, 3, 6> 速度姿勢の計測行列
 */
static inline Eigen::Matrix<double, 3, 6> twistMeasurementMatrix()
{
    Eigen::Matrix<double, 3, 6> C = Eigen::Matrix<double, 3, 6>::Zero();
    C(0, IDX::VX) = 1.0; // for vx
    C(1, IDX::VY) = 1.0; // for vy
    C(2, IDX::WZ) = 1.0; // for wz
    return C;
}

/**
 * @brief 位置姿勢の計測共分散
 *
 * @param covariance 共分散行列
 * @param smoothing_step スムージングステップ
 * @return Eigen::Matrix3d 位置姿勢の計測共分散
 */
static inline Eigen::Matrix3d poseMeasurementCovariance(const std::array<double, 36ul>& covariance, const size_t smoothing_step)
{
    Eigen::Matrix3d R;
    using COV_IDX = XYZRPY_COV_IDX;
    R << covariance.at(COV_IDX::X_X), covariance.at(COV_IDX::X_Y), covariance.at(COV_IDX::X_YAW), covariance.at(COV_IDX::Y_X), covariance.at(COV_IDX::Y_Y), covariance.at(COV_IDX::Y_YAW), covariance.at(COV_IDX::YAW_X), covariance.at(COV_IDX::YAW_Y), covariance.at(COV_IDX::YAW_YAW);
    return R * static_cast<double>(smoothing_step);
}

/**
 * @brief 速度の計測共分散
 *
 * @param covariance 共分散行列
 * @param smoothing_step スムージングステップ
 * @return Eigen::Matrix3d 速度の計測共分散
 */
static inline Eigen::Matrix3d twistMeasurementCovariance(const std::array<double, 36ul>& covariance, const size_t smoothing_step)
{
    Eigen::Matrix3d R;
    using COV_IDX = XYZRPY_COV_IDX;
    R << covariance.at(COV_IDX::X_X), covariance.at(COV_IDX::X_Y), covariance.at(COV_IDX::X_YAW), covariance.at(COV_IDX::Y_X), covariance.at(COV_IDX::Y_Y), covariance.at(COV_IDX::Y_YAW), covariance.at(COV_IDX::YAW_X), covariance.at(COV_IDX::YAW_Y), covariance.at(COV_IDX::YAW_YAW);
    return R * static_cast<double>(smoothing_step);
}

} // namespace tlab