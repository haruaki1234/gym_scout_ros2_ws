/**
 * @file ekf_module.hpp
 * @author Takuma Nakao
 * @brief 拡張カルマンフィルタモジュール
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include "utils/utils.hpp"
#include "utils/kalman_filter.hpp"
#include "utils/time_delay_kalman_filter.hpp"
#include "utils/covariance_index.hpp"

#include "covariance.hpp"
#include "mahalanobis.hpp"
#include "matrix_types.hpp"
#include "measurement.hpp"
#include "numeric.hpp"
#include "state_transition.hpp"
#include "state_index.hpp"
#include "hyper_parameters.hpp"

// clang-format off
#define DEBUG_PRINT_MAT(X) {\
  if (params_.show_debug_info) {std::cout << #X << ": " << X << std::endl;}\
}
// clang-format on

namespace tlab
{

/**
 * @brief 拡張カルマンフィルタモジュール
 *
 */
class EKFModule {
private:
    using PoseWithCovariance = geometry_msgs::msg::PoseWithCovarianceStamped;
    using TwistWithCovariance = geometry_msgs::msg::TwistWithCovarianceStamped;
    using Pose = geometry_msgs::msg::PoseStamped;
    using Twist = geometry_msgs::msg::TwistStamped;

public:
    /**
     * @brief Construct a new EKFModule object
     *
     * @param node ROS2ノードポインタ
     * @param params ハイパーパラメータ
     */
    EKFModule(rclcpp::Node* node, const HyperParameters params) :
        node_(node), params_(params), //
        dim_x_(6),                    // x, y, yaw, vx, vy, wz
        accumulated_delay_times_(params_.extend_state_step, 1.0E15)
    {
        Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_x_, 1);
        Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1.0E15; // for x & y
        P(IDX::YAW, IDX::YAW) = 50.0;                                           // for yaw
        P(IDX::VX, IDX::VX) = 1000.0;                                           // for vx
        P(IDX::VY, IDX::VY) = 1000.0;                                           // for vy
        P(IDX::WZ, IDX::WZ) = 50.0;                                             // for wz

        kalman_filter_.init(X, P, params_.extend_state_step);
    }

    /**
     * @brief 初期化
     *
     * @param initial_pose 初期位置姿勢
     */
    void initialize(const PoseWithCovariance& initial_pose)
    {
        Eigen::MatrixXd X(dim_x_, 1);
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

        X(IDX::X) = initial_pose.pose.pose.position.x;
        X(IDX::Y) = initial_pose.pose.pose.position.y;
        X(IDX::YAW) = tf2::getYaw(initial_pose.pose.pose.orientation);
        X(IDX::VX) = 0.0;
        X(IDX::VY) = 0.0;
        X(IDX::WZ) = 0.0;

        using COV_IDX = XYZRPY_COV_IDX;
        P(IDX::X, IDX::X) = initial_pose.pose.covariance[COV_IDX::X_X];
        P(IDX::Y, IDX::Y) = initial_pose.pose.covariance[COV_IDX::Y_Y];
        P(IDX::YAW, IDX::YAW) = initial_pose.pose.covariance[COV_IDX::YAW_YAW];
        P(IDX::VX, IDX::VX) = 0.01;
        P(IDX::VY, IDX::VY) = 0.01;
        P(IDX::WZ, IDX::WZ) = 0.01;

        kalman_filter_.init(X, P, params_.extend_state_step);
    }

    /**
     * @brief 現在の位置姿勢を取得
     *
     * @param current_time 現在時刻
     * @return geometry_msgs::msg::PoseStamped 現在の位置姿勢
     */
    geometry_msgs::msg::PoseStamped getCurrentPose(const rclcpp::Time& current_time) const
    {
        const double x = kalman_filter_.getXelement(IDX::X);
        const double y = kalman_filter_.getXelement(IDX::Y);
        const double yaw = kalman_filter_.getXelement(IDX::YAW);

        Pose current_ekf_pose;
        current_ekf_pose.header.frame_id = params_.pose_frame_id;
        current_ekf_pose.header.stamp = current_time;
        current_ekf_pose.pose = make_pose(x, y, yaw);
        return current_ekf_pose;
    }

    /**
     * @brief 現在速度を取得
     *
     * @param current_time 現在時刻
     * @return geometry_msgs::msg::TwistStamped 現在の速度
     */
    geometry_msgs::msg::TwistStamped getCurrentTwist(const rclcpp::Time& current_time) const
    {
        const double vx = kalman_filter_.getXelement(IDX::VX);
        const double vy = kalman_filter_.getXelement(IDX::VY);
        const double wz = kalman_filter_.getXelement(IDX::WZ);

        Twist current_ekf_twist;
        current_ekf_twist.header.frame_id = "base_link";
        current_ekf_twist.header.stamp = current_time;
        current_ekf_twist.twist.linear.x = vx;
        current_ekf_twist.twist.linear.y = vy;
        current_ekf_twist.twist.angular.z = wz;
        return current_ekf_twist;
    }
    /**
     * @brief 現在の位置姿勢の共分散を取得
     *
     * @return std::array<double, 36> 現在の位置姿勢の共分散
     */
    std::array<double, 36> getCurrentPoseCovariance() const { return ekfCovarianceToPoseMessageCovariance(kalman_filter_.getLatestP()); }

    /**
     * @brief 現在の速度の共分散を取得
     *
     * @return std::array<double, 36> 現在の速度の共分散
     */
    std::array<double, 36> getCurrentTwistCovariance() const { return ekfCovarianceToTwistMessageCovariance(kalman_filter_.getLatestP()); }

    /**
     * @brief 指定された時間に最も近い遅延時間のインデックスを取得
     *
     * @param target_value 時刻
     * @return size_t 遅延時間のインデックス
     */
    size_t find_closest_delay_time_index(double target_value) const
    {
        // If target_value is too large, return last index + 1
        if (target_value > accumulated_delay_times_.back()) {
            return accumulated_delay_times_.size();
        }

        auto lower = std::lower_bound(accumulated_delay_times_.begin(), accumulated_delay_times_.end(), target_value);

        // If the lower bound is the first element, return its index.
        // If the lower bound is beyond the last element, return the last index.
        // If else, take the closest element.
        if (lower == accumulated_delay_times_.begin()) {
            return 0;
        }
        else if (lower == accumulated_delay_times_.end()) {
            return accumulated_delay_times_.size() - 1;
        }
        else {
            // Compare the target with the lower bound and the previous element.
            auto prev = lower - 1;
            bool is_closer_to_prev = (target_value - *prev) < (*lower - target_value);

            // Return the index of the closer element.
            return is_closer_to_prev ? std::distance(accumulated_delay_times_.begin(), prev) : std::distance(accumulated_delay_times_.begin(), lower);
        }
    }

    /**
     * @brief 遅延時間を蓄積
     *
     * @param dt 経過時間
     */
    void accumulate_delay_time(const double dt)
    {
        // Shift the delay times to the right.
        std::copy_backward(accumulated_delay_times_.begin(), accumulated_delay_times_.end() - 1, accumulated_delay_times_.end());

        // Add a new element (=0) and, and add delay time to the previous elements.
        accumulated_delay_times_.front() = 0.0;
        for (size_t i = 1; i < accumulated_delay_times_.size(); ++i) {
            accumulated_delay_times_[i] += dt;
        }
    }

    /**
     * @brief 遅延保証予測
     *
     * @param dt 経過時間
     */
    void predictWithDelay(const double dt)
    {
        const Eigen::MatrixXd X_curr = kalman_filter_.getLatestX();
        const Eigen::MatrixXd P_curr = kalman_filter_.getLatestP();

        const double proc_cov_vx_d = std::pow(params_.proc_stddev_vx_c * dt, 2.0);
        const double proc_cov_vy_d = std::pow(params_.proc_stddev_vy_c * dt, 2.0);
        const double proc_cov_wz_d = std::pow(params_.proc_stddev_wz_c * dt, 2.0);
        const double proc_cov_yaw_d = std::pow(params_.proc_stddev_yaw_c * dt, 2.0);

        const Vector6d X_next = predictNextState(X_curr, dt);
        const Matrix6d A = createStateTransitionMatrix(X_curr, dt);
        const Matrix6d Q = processNoiseCovariance(proc_cov_yaw_d, proc_cov_vx_d, proc_cov_vy_d, proc_cov_wz_d);
        kalman_filter_.predictWithDelay(X_next, A, Q);
    }

    /**
     * @brief 位置姿勢の計測更新
     *
     * @param pose 位置姿勢
     * @param t_curr 位置姿勢計測時刻
     * @retval true 成功
     * @retval false 失敗
     */
    bool measurementUpdatePose(const PoseWithCovariance& pose, const rclcpp::Time& t_curr)
    {
        if (pose.header.frame_id != params_.pose_frame_id) {
            RCLCPP_WARN(node_->get_logger(), "pose frame_id is %s, but pose_frame is set as %s. They must be same.", pose.header.frame_id.c_str(), params_.pose_frame_id.c_str());
        }
        const Eigen::MatrixXd X_curr = kalman_filter_.getLatestX();
        DEBUG_PRINT_MAT(X_curr.transpose());

        constexpr int dim_y = 3; // pos_x, pos_y, yaw, depending on Pose output

        /* Calculate delay step */
        double delay_time = (t_curr - pose.header.stamp).seconds() + params_.pose_additional_delay;
        if (delay_time < 0.0) {
            RCLCPP_WARN(node_->get_logger(), "pose delay time is negative. delay time is %f", delay_time);
        }

        delay_time = std::max(delay_time, 0.0);

        const int delay_step = static_cast<int>(find_closest_delay_time_index(delay_time));

        if (delay_step >= params_.extend_state_step) {
            RCLCPP_WARN(node_->get_logger(), "pose delay step is %d. delay step threshold is %d", delay_step, params_.extend_state_step);
            return false;
        }

        /* Since the kalman filter cannot handle the rotation angle directly,
          offset the yaw angle so that the difference from the yaw angle that ekf holds internally is less
          than 2 pi. */
        double yaw = tf2::getYaw(pose.pose.pose.orientation);
        const double ekf_yaw = kalman_filter_.getXelement(delay_step * dim_x_ + IDX::YAW);
        const double yaw_error = normalizeYaw(yaw - ekf_yaw); // normalize the error not to exceed 2 pi
        yaw = yaw_error + ekf_yaw;

        /* Set measurement matrix */
        Eigen::MatrixXd y(dim_y, 1);
        y << pose.pose.pose.position.x, pose.pose.pose.position.y, yaw;

        if (hasNan(y) || hasInf(y)) {
            RCLCPP_WARN(node_->get_logger(), "[EKF] pose measurement matrix includes NaN of Inf. ignore update. check pose message.");
            return false;
        }

        /* Gate */
        const Eigen::Vector3d y_ekf(kalman_filter_.getXelement(delay_step * dim_x_ + IDX::X), kalman_filter_.getXelement(delay_step * dim_x_ + IDX::Y), ekf_yaw);
        const Eigen::MatrixXd P_curr = kalman_filter_.getLatestP();
        const Eigen::MatrixXd P_y = P_curr.block(0, 0, dim_y, dim_y);

        const double distance = mahalanobis(y_ekf, y, P_y);
        if (distance > params_.pose_gate_dist) {
            RCLCPP_WARN(node_->get_logger(), "Ignore the measurement data. pose mahalanobis distance is %f. mahalanobis distance threshold is %f", distance, params_.pose_gate_dist);
            return false;
        }

        DEBUG_PRINT_MAT(y.transpose());
        DEBUG_PRINT_MAT(y_ekf.transpose());
        DEBUG_PRINT_MAT((y - y_ekf).transpose());

        const Eigen::Matrix<double, 3, 6> C = poseMeasurementMatrix();
        const Eigen::Matrix3d R = poseMeasurementCovariance(pose.pose.covariance, params_.pose_smoothing_steps);

        kalman_filter_.updateWithDelay(y, C, R, delay_step);

        // debug
        const Eigen::MatrixXd X_result = kalman_filter_.getLatestX();
        DEBUG_PRINT_MAT(X_result.transpose());
        DEBUG_PRINT_MAT((X_result - X_curr).transpose());

        return true;
    }

    /**
     * @brief 速度の計測更新
     *
     * @param twist 速度
     * @param t_curr 速度計測時刻
     * @retval true 成功
     * @retval false 失敗
     */
    bool measurementUpdateTwist(const TwistWithCovariance& twist, const rclcpp::Time& t_curr)
    {
        const Eigen::MatrixXd X_curr = kalman_filter_.getLatestX();
        DEBUG_PRINT_MAT(X_curr.transpose());

        constexpr int dim_y = 3; // vx, vy, wz

        /* Calculate delay step */
        double delay_time = (t_curr - twist.header.stamp).seconds() + params_.twist_additional_delay;
        if (delay_time < 0.0) {
            RCLCPP_WARN(node_->get_logger(), "twist delay time is negative. delay time is %f", delay_time);
        }
        delay_time = std::max(delay_time, 0.0);

        const int delay_step = static_cast<int>(find_closest_delay_time_index(delay_time));

        if (delay_step >= params_.extend_state_step) {
            RCLCPP_WARN(node_->get_logger(), "twist delay step is %d. delay step threshold is %d", delay_step, params_.extend_state_step);
            return false;
        }

        /* Set measurement matrix */
        Eigen::MatrixXd y(dim_y, 1);
        y << twist.twist.twist.linear.x, twist.twist.twist.linear.y, twist.twist.twist.angular.z;

        if (hasNan(y) || hasInf(y)) {
            RCLCPP_WARN(node_->get_logger(), "[EKF] twist measurement matrix includes NaN of Inf. ignore update. check twist message.");
            return false;
        }

        const Eigen::Vector3d y_ekf(kalman_filter_.getXelement(delay_step * dim_x_ + IDX::VX), kalman_filter_.getXelement(delay_step * dim_x_ + IDX::VY), kalman_filter_.getXelement(delay_step * dim_x_ + IDX::WZ));
        const Eigen::MatrixXd P_curr = kalman_filter_.getLatestP();
        const Eigen::MatrixXd P_y = P_curr.block(3, 3, dim_y, dim_y);

        const double distance = mahalanobis(y_ekf, y, P_y);
        if (distance > params_.twist_gate_dist) {
            RCLCPP_WARN(node_->get_logger(), "Ignore the measurement data. twist mahalanobis distance is %f. mahalanobis distance threshold is %f", distance, params_.twist_gate_dist);
            return false;
        }

        DEBUG_PRINT_MAT(y.transpose());
        DEBUG_PRINT_MAT(y_ekf.transpose());
        DEBUG_PRINT_MAT((y - y_ekf).transpose());

        const Eigen::Matrix<double, 3, 6> C = twistMeasurementMatrix();
        const Eigen::Matrix3d R = twistMeasurementCovariance(twist.twist.covariance, params_.twist_smoothing_steps);
        kalman_filter_.updateWithDelay(y, C, R, delay_step);

        // debug
        const Eigen::MatrixXd X_result = kalman_filter_.getLatestX();
        DEBUG_PRINT_MAT(X_result.transpose());
        DEBUG_PRINT_MAT((X_result - X_curr).transpose());

        return true;
    }

private:
    //! ROS2ノードポインタ
    rclcpp::Node* node_;
    //! ハイパーパラメータ
    const HyperParameters params_;

    //! 遅延保証拡張カルマンフィルタ
    TimeDelayKalmanFilter kalman_filter_;

    //! 状態量の次元
    const int dim_x_;
    //! 遅延時間の蓄積ベクトル
    std::vector<double> accumulated_delay_times_;
};

} // namespace tlab