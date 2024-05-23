/**
 * @file localization_component.hpp
 * @author Takuma Nakao
 * @brief センサフュージョンによるロボットの位置推定を行うコンポーネント
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <chrono>
#include <algorithm>
#include <Eigen/Core>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "utils/utils.hpp"
#include "utils/stop_watch.hpp"
#include "utils/covariance_index.hpp"

#include "ekf_module.hpp"
#include "aged_object_queue.hpp"

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define DEBUG_INFO(...) {if (params_.show_debug_info) {RCLCPP_INFO(__VA_ARGS__);}}
// clang-format on

namespace tlab
{

class Localization : public rclcpp::Node {
private:
    const HyperParameters params_;

    double ekf_dt_;

    AgedObjectQueue<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> pose_queue_;
    AgedObjectQueue<geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr> twist_queue_;

    tf2_ros::TransformBroadcaster broadcaster_;

    StopWatch<std::chrono::milliseconds> stop_watch_;
    std::shared_ptr<const rclcpp::Time> last_predict_time_;

    std::unique_ptr<EKFModule> ekf_module_;

    /* process noise variance for discrete model */
    double proc_cov_yaw_d_;      //!< @brief  discrete yaw process noise
    double proc_cov_yaw_bias_d_; //!< @brief  discrete yaw bias process noise
    double proc_cov_vx_d_;       //!< @brief  discrete process noise in d_vx=0
    double proc_cov_vy_d_;       //!< @brief  discrete process noise in d_vy=0
    double proc_cov_wz_d_;       //!< @brief  discrete process noise in d_wz=0

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_cov_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_biased_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_biased_pose_cov_;

public:
    Localization(const rclcpp::NodeOptions& options) : Localization("", options) {}
    Localization(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
        Node("localization_node", name_space, options), //
        params_(this),                                  //
        ekf_dt_(params_.ekf_dt),                        //
        pose_queue_(params_.pose_smoothing_steps),      //
        twist_queue_(params_.twist_smoothing_steps),    //
        broadcaster_(this)                              //
    {
        using namespace std::chrono_literals;

        proc_cov_vx_d_ = std::pow(params_.proc_stddev_vx_c * ekf_dt_, 2.0);
        proc_cov_vy_d_ = std::pow(params_.proc_stddev_vy_c * ekf_dt_, 2.0);
        proc_cov_wz_d_ = std::pow(params_.proc_stddev_wz_c * ekf_dt_, 2.0);
        proc_cov_yaw_d_ = std::pow(params_.proc_stddev_yaw_c * ekf_dt_, 2.0);

        ekf_module_ = std::make_unique<EKFModule>(this, params_);

        {
            declare_parameter("start_pos", std::vector<double>{0, 0, 0});
            auto initial_pos = get_parameter("start_pos").as_double_array();
            geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
            initial_pose.pose.pose = make_pose(initial_pos[0], initial_pos[1], initial_pos[2]);
            ekf_module_->initialize(initial_pose);
        }

        pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose", 1);
        pub_pose_cov_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf_pose_with_covariance", 1);
        pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("ekf_odom", 1);
        pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("ekf_twist", 1);
        pub_twist_cov_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("ekf_twist_with_covariance", 1);

        static auto sub_initialpose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1, [&](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { ekf_module_->initialize(*msg); });
        static auto sub_pose_with_cov = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("umap_pos", 1, [&](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { pose_queue_.push(msg); });
        static auto sub_twist_with_cov = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1, [&](nav_msgs::msg::Odometry::SharedPtr msg) {
            auto twist_with_covariance_stamped = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
            twist_with_covariance_stamped->header = msg->header;
            twist_with_covariance_stamped->twist = msg->twist;
            twist_with_covariance_stamped->twist.covariance[XYZRPY_COV_IDX::X_X] = std::pow(params_.twist_stddev_vx_c, 2.0);
            twist_with_covariance_stamped->twist.covariance[XYZRPY_COV_IDX::Y_Y] = std::pow(params_.twist_stddev_vy_c, 2.0);
            twist_with_covariance_stamped->twist.covariance[XYZRPY_COV_IDX::YAW_YAW] = std::pow(params_.twist_stddev_wz_c, 2.0);
            twist_queue_.push(twist_with_covariance_stamped);
        });

        if (params_.publish_tf_) {
            static auto timer_tf = this->create_wall_timer(rclcpp::Rate(params_.tf_rate_).period(), [&]() {
                if (params_.pose_frame_id == "") {
                    return;
                }

                const rclcpp::Time current_time = this->get_clock()->now();

                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped.header.stamp = current_time;
                transform_stamped.header.frame_id = params_.pose_frame_id;
                transform_stamped.child_frame_id = "base_link";
                transform_stamped.transform = make_geometry_transform(make_eigen_vector3d(ekf_module_->getCurrentPose(current_time).pose));
                broadcaster_.sendTransform(transform_stamped);
            });
        }

        static auto timer_control = this->create_wall_timer(1s * ekf_dt_, [&]() {
            const rclcpp::Time current_time = this->now();

            DEBUG_INFO(get_logger(), "========================= timer called =========================");

            /* update predict frequency with measured timer rate */
            updatePredictFrequency(current_time);

            /* predict model in EKF */
            stop_watch_.tic();
            DEBUG_INFO(get_logger(), "------------------------- start prediction -------------------------");
            ekf_module_->predictWithDelay(ekf_dt_);
            DEBUG_INFO(get_logger(), "[EKF] predictKinematicsModel calc time = %f [ms]", stop_watch_.toc());
            DEBUG_INFO(get_logger(), "------------------------- end prediction -------------------------\n");

            if (!pose_queue_.empty()) {
                DEBUG_INFO(get_logger(), "------------------------- start Pose -------------------------");
                stop_watch_.tic();

                // save the initial size because the queue size can change in the loop
                const auto t_curr = current_time;
                const size_t n = pose_queue_.size();
                for (size_t i = 0; i < n; ++i) {
                    const auto pose = pose_queue_.pop_increment_age();
                    ekf_module_->measurementUpdatePose(*pose, t_curr);
                }
                DEBUG_INFO(get_logger(), "[EKF] measurementUpdatePose calc time = %f [ms]", stop_watch_.toc());
                DEBUG_INFO(get_logger(), "------------------------- end Pose -------------------------\n");
            }

            if (!twist_queue_.empty()) {
                DEBUG_INFO(get_logger(), "------------------------- start Twist -------------------------");
                stop_watch_.tic();

                // save the initial size because the queue size can change in the loop
                const auto t_curr = current_time;
                const size_t n = twist_queue_.size();
                for (size_t i = 0; i < n; ++i) {
                    const auto twist = twist_queue_.pop_increment_age();
                    ekf_module_->measurementUpdateTwist(*twist, t_curr);
                }
                DEBUG_INFO(get_logger(), "[EKF] measurementUpdateTwist calc time = %f [ms]", stop_watch_.toc());
                DEBUG_INFO(get_logger(), "------------------------- end Twist -------------------------\n");
            }

            const geometry_msgs::msg::PoseStamped current_ekf_pose = ekf_module_->getCurrentPose(current_time);
            const geometry_msgs::msg::TwistStamped current_ekf_twist = ekf_module_->getCurrentTwist(current_time);

            /* publish ekf result */
            publishEstimateResult(current_ekf_pose, current_ekf_twist);
        });
    }

private:
    void updatePredictFrequency(const rclcpp::Time& current_time)
    {
        if (last_predict_time_) {
            if (current_time < *last_predict_time_) {
                RCLCPP_WARN(get_logger(), "Detected jump back in time");
            }
            else {
                /* Measure dt */
                ekf_dt_ = (current_time - *last_predict_time_).seconds();
                DEBUG_INFO(get_logger(), "[EKF] update ekf_dt_ to %f seconds (= %f hz)", ekf_dt_, 1 / ekf_dt_);

                if (ekf_dt_ > 10.0) {
                    ekf_dt_ = 10.0;
                    RCLCPP_WARN(get_logger(), "Large ekf_dt_ detected!! (%f sec) Capped to 10.0 seconds", ekf_dt_);
                }
                else if (ekf_dt_ > params_.pose_smoothing_steps / params_.ekf_rate) {
                    RCLCPP_WARN(get_logger(), "EKF period may be too slow to finish pose smoothing!! (%f sec) ", ekf_dt_);
                }

                /* Register dt and accumulate time delay */
                ekf_module_->accumulate_delay_time(ekf_dt_);

                /* Update discrete proc_cov*/
                proc_cov_vx_d_ = std::pow(params_.proc_stddev_vx_c * ekf_dt_, 2.0);
                proc_cov_vy_d_ = std::pow(params_.proc_stddev_vy_c * ekf_dt_, 2.0);
                proc_cov_wz_d_ = std::pow(params_.proc_stddev_wz_c * ekf_dt_, 2.0);
                proc_cov_yaw_d_ = std::pow(params_.proc_stddev_yaw_c * ekf_dt_, 2.0);
            }
        }
        last_predict_time_ = std::make_shared<const rclcpp::Time>(current_time);
    }

    void publishEstimateResult(const geometry_msgs::msg::PoseStamped& current_ekf_pose, const geometry_msgs::msg::TwistStamped& current_ekf_twist)
    {
        /* publish latest pose */
        pub_pose_->publish(current_ekf_pose);

        /* publish latest pose with covariance */
        geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
        pose_cov.header.stamp = current_ekf_pose.header.stamp;
        pose_cov.header.frame_id = current_ekf_pose.header.frame_id;
        pose_cov.pose.pose = current_ekf_pose.pose;
        pose_cov.pose.covariance = ekf_module_->getCurrentPoseCovariance();
        pub_pose_cov_->publish(pose_cov);

        /* publish latest twist */
        pub_twist_->publish(current_ekf_twist);

        /* publish latest twist with covariance */
        geometry_msgs::msg::TwistWithCovarianceStamped twist_cov;
        twist_cov.header.stamp = current_ekf_twist.header.stamp;
        twist_cov.header.frame_id = current_ekf_twist.header.frame_id;
        twist_cov.twist.twist = current_ekf_twist.twist;
        twist_cov.twist.covariance = ekf_module_->getCurrentTwistCovariance();
        pub_twist_cov_->publish(twist_cov);

        /* publish latest odometry */
        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = current_ekf_pose.header.stamp;
        odometry.header.frame_id = current_ekf_pose.header.frame_id;
        odometry.child_frame_id = "base_link";
        odometry.pose = pose_cov.pose;
        odometry.twist = twist_cov.twist;
        pub_odom_->publish(odometry);
    }
};

} // namespace tlab