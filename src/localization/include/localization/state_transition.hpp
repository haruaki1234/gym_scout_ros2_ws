/**
 * @file state_transition.hpp
 * @author Takuma Nakao
 * @brief カルマンフィルタのモデル更新
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "matrix_types.hpp"
#include "state_index.hpp"

namespace tlab
{

/**
 * @brief 角度の正規化
 *
 * @param yaw 角度
 * @return double 正規化された角度
 */
double normalizeYaw(const double& yaw) { return std::atan2(std::sin(yaw), std::cos(yaw)); }

/**
 * @brief 次の状態を予測
 *
 * @param X_curr 現在の状態
 * @param dt 時間差分
 * @return Vector6d 次の状態
 */
/*  == Nonlinear model ==
 *
 * x_{k+1}   = x_k + (vx_k * cos(yaw_k) - vy_k * sin(yaw_k)) * dt
 * y_{k+1}   = y_k + (vx_k * sin(yaw_k) + vy_k * cos(yaw_k)) * dt
 * yaw_{k+1} = yaw_k + (wz_k) * dt
 * vx_{k+1}  = vx_k
 * vy_{k+1}  = vy_k
 * wz_{k+1}  = wz_k
 */
Vector6d predictNextState(const Vector6d& X_curr, const double dt)
{
    const double x = X_curr(IDX::X);
    const double y = X_curr(IDX::Y);
    const double yaw = X_curr(IDX::YAW);
    const double vx = X_curr(IDX::VX);
    const double vy = X_curr(IDX::VY);
    const double wz = X_curr(IDX::WZ);

    Vector6d X_next;
    X_next(IDX::X) = x + (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt; // dx = vx * cos(yaw) - vy * sin(yaw)
    X_next(IDX::Y) = y + (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt; // dy = vx * sin(yaw) + vy * cos(yaw)
    X_next(IDX::YAW) = normalizeYaw(yaw + wz * dt);                      // dyaw = omega + omega_bias
    X_next(IDX::VX) = vx;
    X_next(IDX::VY) = vy;
    X_next(IDX::WZ) = wz;
    return X_next;
}

/**
 * @brief 状態遷移行列を計算
 *
 * @param X_curr 現在の状態
 * @param dt 時間差分
 * @return Matrix6d 状態遷移行列
 */
/*  == Linearized model ==
 *
 * A = [ 1, 0, (-vx*sin(yaw)-vy*cos(yaw))*dt, cos(yaw)*dt, -sin(yaw)*dt,   0]
 *     [ 0, 1,  (vx*cos(yaw)-vy*sin(yaw))*dt, sin(yaw)*dt,  cos(yaw)*dt,   0]
 *     [ 0, 0,                             1,           0,            0,  dt]
 *     [ 0, 0,                             0,           1,            0,   0]
 *     [ 0, 0,                             0,           0,            1,   0]
 *     [ 0, 0,                             0,           0,            0,   1]
 */
Matrix6d createStateTransitionMatrix(const Vector6d& X_curr, const double dt)
{
    const double yaw = X_curr(IDX::YAW);
    const double vx = X_curr(IDX::VX);
    const double vy = X_curr(IDX::VY);

    Matrix6d A = Matrix6d::Identity();
    A(IDX::X, IDX::YAW) = (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
    A(IDX::X, IDX::VX) = std::cos(yaw) * dt;
    A(IDX::X, IDX::VY) = -std::sin(yaw) * dt;
    A(IDX::Y, IDX::YAW) = (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
    A(IDX::Y, IDX::VX) = std::sin(yaw) * dt;
    A(IDX::Y, IDX::VY) = std::cos(yaw) * dt;
    A(IDX::YAW, IDX::WZ) = dt;
    return A;
}

/**
 * @brief プロセスノイズの共分散行列を計算
 *
 * @param proc_cov_yaw_d 角速度の分散
 * @param proc_cov_vx_d 速度の分散
 * @param proc_cov_vy_d 速度の分散
 * @param proc_cov_wz_d 角速度の分散
 * @return Matrix6d プロセスノイズの共分散行列
 */
Matrix6d processNoiseCovariance(const double proc_cov_yaw_d, const double proc_cov_vx_d, const double proc_cov_vy_d, const double proc_cov_wz_d)
{
    Matrix6d Q = Matrix6d::Zero();
    Q(IDX::X, IDX::X) = 0.0;
    Q(IDX::Y, IDX::Y) = 0.0;
    Q(IDX::YAW, IDX::YAW) = proc_cov_yaw_d; // for yaw
    Q(IDX::VX, IDX::VX) = proc_cov_vx_d;    // for vx
    Q(IDX::VY, IDX::VY) = proc_cov_vy_d;    // for vy
    Q(IDX::WZ, IDX::WZ) = proc_cov_wz_d;    // for wz
    return Q;
}

} // namespace tlab