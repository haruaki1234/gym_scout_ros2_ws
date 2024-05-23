/**
 * @file time_delay_kalman_filter.hpp
 * @author Takuma Nakao
 * @brief 遅延保証付きカルマンフィルタ
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "kalman_filter.hpp"

#include <Eigen/Core>
#include <Eigen/LU>

#include <iostream>

namespace tlab
{

class TimeDelayKalmanFilter : public KalmanFilter {
public:
    /**
     * @brief No initialization constructor.
     */
    TimeDelayKalmanFilter() {}

    /**
     * @brief initialization of kalman filter
     * @param x initial state
     * @param P0 initial covariance of estimated state
     * @param max_delay_step Maximum number of delay steps, which determines the dimension of the
     * extended kalman filter
     */
    void init(const Eigen::MatrixXd& x, const Eigen::MatrixXd& P0, const int max_delay_step)
    {
        max_delay_step_ = max_delay_step;
        dim_x_ = x.rows();
        dim_x_ex_ = dim_x_ * max_delay_step;

        x_ = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
        P_ = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);

        for (int i = 0; i < max_delay_step_; ++i) {
            x_.block(i * dim_x_, 0, dim_x_, 1) = x;
            P_.block(i * dim_x_, i * dim_x_, dim_x_, dim_x_) = P0;
        }
    }

    /**
     * @brief get latest time estimated state
     */
    Eigen::MatrixXd getLatestX() const { return x_.block(0, 0, dim_x_, 1); }

    /**
     * @brief get latest time estimation covariance
     */
    Eigen::MatrixXd getLatestP() const { return P_.block(0, 0, dim_x_, dim_x_); }

    /**
     * @brief calculate kalman filter covariance by precision model with time delay. This is mainly
     * for EKF of nonlinear process model.
     * @param x_next predicted state by prediction model
     * @param A coefficient matrix of x for process model
     * @param Q covariance matrix for process model
     */
    bool predictWithDelay(const Eigen::MatrixXd& x_next, const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q)
    {
        /*
         * time delay model:
         *
         *     [A   0   0]      [P11   P12   P13]      [Q   0   0]
         * A = [I   0   0], P = [P21   P22   P23], Q = [0   0   0]
         *     [0   I   0]      [P31   P32   P33]      [0   0   0]
         *
         * covariance calculation in prediction : P = A * P * A' + Q
         *
         *     [A*P11*A'*+Q  A*P11  A*P12]
         * P = [     P11*A'    P11    P12]
         *     [     P21*A'    P21    P22]
         */

        const int d_dim_x = dim_x_ex_ - dim_x_;

        /* slide states in the time direction */
        Eigen::MatrixXd x_tmp = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
        x_tmp.block(0, 0, dim_x_, 1) = x_next;
        x_tmp.block(dim_x_, 0, d_dim_x, 1) = x_.block(0, 0, d_dim_x, 1);
        x_ = x_tmp;

        /* update P with delayed measurement A matrix structure */
        Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);
        P_tmp.block(0, 0, dim_x_, dim_x_) = A * P_.block(0, 0, dim_x_, dim_x_) * A.transpose() + Q;
        P_tmp.block(0, dim_x_, dim_x_, d_dim_x) = A * P_.block(0, 0, dim_x_, d_dim_x);
        P_tmp.block(dim_x_, 0, d_dim_x, dim_x_) = P_.block(0, 0, d_dim_x, dim_x_) * A.transpose();
        P_tmp.block(dim_x_, dim_x_, d_dim_x, d_dim_x) = P_.block(0, 0, d_dim_x, d_dim_x);
        P_ = P_tmp;

        return true;
    }

    /**
     * @brief calculate kalman filter covariance by measurement model with time delay. This is mainly
     * for EKF of nonlinear process model.
     * @param y measured values
     * @param C coefficient matrix of x for measurement model
     * @param R covariance matrix for measurement model
     * @param delay_step measurement delay
     */
    bool updateWithDelay(const Eigen::MatrixXd& y, const Eigen::MatrixXd& C, const Eigen::MatrixXd& R, const int delay_step)
    {
        if (delay_step >= max_delay_step_) {
            std::cerr << "delay step is larger than max_delay_step. ignore update." << std::endl;
            return false;
        }

        const int dim_y = y.rows();

        /* set measurement matrix */
        Eigen::MatrixXd C_ex = Eigen::MatrixXd::Zero(dim_y, dim_x_ex_);
        C_ex.block(0, dim_x_ * delay_step, dim_y, dim_x_) = C;

        /* update */
        if (!update(y, C_ex, R)) {
            return false;
        }

        return true;
    }

private:
    int max_delay_step_; //!< @brief maximum number of delay steps
    int dim_x_;          //!< @brief dimension of latest state
    int dim_x_ex_;       //!< @brief dimension of extended state with dime delay
};

} // namespace tlab