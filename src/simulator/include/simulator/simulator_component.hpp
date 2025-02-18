/**
 * @file simulator_component.hpp
 * @author Takuma Nakao
 * @brief シミュレータコンポーネント
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <random>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Dense>

#include "utils/utils.hpp"
#include "utils/velocity_limit_filter.hpp"
#include "utils/covariance_index.hpp"

namespace tlab
{

/**
 * @brief シミュレータノード
 *
 */
class Simulator : public rclcpp::Node {
private:
    //! シミュレーション周期
    double dt_ = 0.01;
    //! ロボット最大速度
    double max_vel_ = 1.0;
    //! ロボット最大加速度
    double max_acc_ = 1.0;
    //! ロボット最大角速度
    double max_ang_vel_ = 1.0;
    //! ロボット最大角加速度
    double max_ang_acc_ = 1.0;

    //! VGMリクエスト周期
    double umap_request_period_ = 0.1;
    //! VGM位置推定遅延
    double umap_localization_delay_ = 0.0;

    //! ロボット目標速度
    Eigen::Vector3d target_vel_;
    //! ロボット真値速度
    Eigen::Vector3d truth_vel_;
    //! ロボット真値位置
    Eigen::Vector3d truth_pos_;
    //! ロボット現在位置
    Eigen::Vector3d current_pos_;
    //! ロボット現在速度
    Eigen::Vector3d current_vel_;

    //! ロボット推定位置
    Eigen::Vector3d localization_pos_;

    //! VGMリクエスト時刻
    std::optional<rclcpp::Time> umap_request_time_ = std::nullopt;

    //! VGM推定位置シミュレーション結果
    Eigen::Vector3d umap_pos_;
    //! VGMリクエスト時の推定ロボット位置
    Eigen::Vector3d umap_localization_pos_;

    //! ロボット初期位置
    Eigen::Vector3d initial_pos_;

public:
    /**
     * @brief Construct a new Simulator object
     *
     * @param options
     */
    Simulator(const rclcpp::NodeOptions& options) : Simulator("", options) {}
    /**
     * @brief Construct a new Simulator object
     *
     * @param name_space
     * @param options
     */
    Simulator(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("simulator_node", name_space, options)
    {
        using namespace std::chrono_literals;

        declare_parameter<double>("dt", 0.01);
        dt_ = get_parameter("dt").as_double();
        declare_parameter<double>("max_vel", 1.0);
        max_vel_ = get_parameter("max_vel").as_double();
        declare_parameter<double>("max_acc", 1.0);
        max_acc_ = get_parameter("max_acc").as_double();
        declare_parameter<double>("max_ang_vel", 1.0);
        max_ang_vel_ = get_parameter("max_ang_vel").as_double();
        declare_parameter<double>("max_ang_acc", 1.0);
        max_ang_acc_ = get_parameter("max_ang_acc").as_double();

        declare_parameter<double>("init_x", 0.0);
        declare_parameter<double>("init_y", 0.0);
        declare_parameter<double>("init_yaw", 0.0);

        initial_pos_.x() = get_parameter("init_x").as_double();
        initial_pos_.y() = get_parameter("init_y").as_double();
        initial_pos_.z() = get_parameter("init_yaw").as_double();

        truth_pos_ = initial_pos_;
        truth_vel_ = Eigen::Vector3d::Zero();

        current_pos_ = Eigen::Vector3d::Zero();
        current_vel_ = Eigen::Vector3d::Zero();

        target_vel_ = Eigen::Vector3d::Zero();

        static std::random_device seed_gen;
        static std::mt19937 engine(seed_gen());

        declare_parameter<double>("vel_x_mean", 1.0);
        declare_parameter<double>("vel_x_stddev", 0.01);
        declare_parameter<double>("vel_y_mean", 1.0);
        declare_parameter<double>("vel_y_stddev", 0.01);
        declare_parameter<double>("vel_yaw_mean", 1.0);
        declare_parameter<double>("vel_yaw_stddev", 0.01);
        static std::normal_distribution<> vel_x_dist(get_parameter("vel_x_mean").as_double(), get_parameter("vel_x_stddev").as_double());
        static std::normal_distribution<> vel_y_dist(get_parameter("vel_y_mean").as_double(), get_parameter("vel_y_stddev").as_double());
        static std::normal_distribution<> vel_th_dist(get_parameter("vel_yaw_mean").as_double(), get_parameter("vel_yaw_stddev").as_double());

        declare_parameter<double>("umap_request_period", 0.1);
        umap_request_period_ = get_parameter("umap_request_period").as_double();
        declare_parameter<double>("umap_localization_delay", 0.0);
        umap_localization_delay_ = get_parameter("umap_localization_delay").as_double();

        declare_parameter<double>("umap_pos_x_mean", 0.0);
        declare_parameter<double>("umap_pos_x_stddev", 0.01);
        declare_parameter<double>("umap_pos_y_mean", 0.0);
        declare_parameter<double>("umap_pos_y_stddev", 0.01);
        declare_parameter<double>("umap_pos_yaw_mean", 0.0);
        declare_parameter<double>("umap_pos_yaw_stddev", 0.01);
        static std::normal_distribution<> umap_pos_x_dist(get_parameter("umap_pos_x_mean").as_double(), get_parameter("umap_pos_x_stddev").as_double());
        static std::normal_distribution<> umap_pos_y_dist(get_parameter("umap_pos_y_mean").as_double(), get_parameter("umap_pos_y_stddev").as_double());
        static std::normal_distribution<> umap_pos_th_dist(get_parameter("umap_pos_yaw_mean").as_double(), get_parameter("umap_pos_yaw_stddev").as_double());

        static std::uniform_real_distribution<> rand_01(0.0, 1.0);

        static ClampVelLimitFilter vx_filter(-max_vel_, max_vel_, max_acc_, dt_);
        static ClampVelLimitFilter vy_filter(-max_vel_, max_vel_, max_acc_, dt_);
        static ClampVelLimitFilter vth_filter(-max_ang_vel_, max_ang_vel_, max_ang_acc_, dt_);
        vx_filter.reset(0);
        vy_filter.reset(0);
        vth_filter.reset(0);

        static auto truth_odom_pub = create_publisher<nav_msgs::msg::Odometry>("truth_odom", rclcpp::QoS(10).reliable());
        static auto odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10).reliable());

        static auto umap_pos_pub = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("umap_pos", rclcpp::QoS(10).reliable());

        static auto set_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", rclcpp::QoS(10).reliable(), [&](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            initial_pos_ = make_eigen_vector3d(msg->pose.pose);
            truth_pos_ = initial_pos_;
        });

        static auto target_vel_sub = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 5, [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            vx_filter.set_input(msg->linear.x);
            vy_filter.set_input(msg->linear.y);
            vth_filter.set_input(msg->angular.z);
        });

        static auto localization_sub = create_subscription<nav_msgs::msg::Odometry>("ekf_odom", rclcpp::QoS(10).reliable(), [&](const nav_msgs::msg::Odometry::SharedPtr msg) { localization_pos_ = make_eigen_vector3d(msg->pose.pose); });

        static auto umap_sim_timer = create_wall_timer(1s * umap_request_period_, [&]() {
            umap_localization_pos_ = localization_pos_;
            umap_pos_ = truth_pos_;
            // double outlier_gain = 1.0;
            // if (rand_01(engine) < 0.1) {
            //     outlier_gain = 20.0;
            // }
            umap_pos_.x() += umap_pos_x_dist(engine);
            umap_pos_.y() += umap_pos_y_dist(engine);
            umap_pos_.z() += umap_pos_th_dist(engine);

            umap_request_time_ = this->get_clock()->now();
        });

        static auto timer = create_wall_timer(1s * dt_, [&]() {
            if (umap_request_time_ && this->get_clock()->now().seconds() - umap_request_time_.value().seconds() > umap_localization_delay_) {
                // auto localization_diff = localization_pos_ - umap_localization_pos_;
                // umap_pos_.head<2>() += rotate_2d(localization_diff.head<2>(), umap_pos_.z() - umap_localization_pos_.z());
                // umap_pos_.z() += localization_diff.z();

                geometry_msgs::msg::PoseWithCovarianceStamped umap_pos_msg;
                umap_pos_msg.header.frame_id = "map";
                umap_pos_msg.header.stamp = *umap_request_time_;
                umap_pos_msg.pose.pose = make_pose(umap_pos_);
                umap_pos_msg.pose.covariance[XYZRPY_COV_IDX::X_X] = get_parameter("umap_pos_x_stddev").as_double();
                umap_pos_msg.pose.covariance[XYZRPY_COV_IDX::Y_Y] = get_parameter("umap_pos_y_stddev").as_double();
                umap_pos_msg.pose.covariance[XYZRPY_COV_IDX::YAW_YAW] = get_parameter("umap_pos_yaw_stddev").as_double();
                umap_pos_pub->publish(umap_pos_msg);

                umap_request_time_ = std::nullopt;
            }
            {
                truth_vel_.x() = vx_filter.filtering();
                truth_vel_.y() = vy_filter.filtering();
                truth_vel_.z() = vth_filter.filtering();
                auto rotate_vel = rotate_2d(truth_vel_.head<2>(), truth_pos_.z());
                truth_pos_.x() += rotate_vel.x() * dt_;
                truth_pos_.y() += rotate_vel.y() * dt_;
                truth_pos_.z() += truth_vel_.z() * dt_;
            }
            {
                nav_msgs::msg::Odometry truth_odom_msg;
                truth_odom_msg.header.frame_id = "map";
                truth_odom_msg.header.stamp = this->get_clock()->now();
                truth_odom_msg.pose.pose = make_pose(truth_pos_);
                truth_odom_msg.twist.twist = make_twist(truth_vel_);
                truth_odom_pub->publish(truth_odom_msg);
            }
            {
                current_vel_.x() = truth_vel_.x() * vel_x_dist(engine);
                current_vel_.y() = truth_vel_.y() * vel_y_dist(engine);
                current_vel_.z() = truth_vel_.z() * vel_th_dist(engine);
                auto rotate_vel = rotate_2d(current_vel_.head<2>(), current_pos_.z());
                current_pos_.x() += rotate_vel.x() * dt_;
                current_pos_.y() += rotate_vel.y() * dt_;
                current_pos_.z() += current_vel_.z() * dt_;
            }
            {
                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.frame_id = "initial_pos";
                odom_msg.header.stamp = this->get_clock()->now();
                odom_msg.pose.pose = make_pose(current_pos_);
                odom_msg.twist.twist = make_twist(current_vel_);
                odom_pub->publish(odom_msg);
            }
        });
    }
};

} // namespace tlab