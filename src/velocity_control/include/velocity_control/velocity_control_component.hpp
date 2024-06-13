/**
 * @file velocity_control_component.hpp
 * @author Takuma Nakao
 * @brief 速度制御コンポーネント
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <random>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "utils/utils.hpp"

namespace tlab
{

/**
 * @brief 速度制御ノード
 *
 */
class VelocityControl : public rclcpp::Node {
private:
    //! 目標速度
    Eigen::Vector3d target_vel_ = Eigen::Vector3d::Zero();
    //! 現在速度
    Eigen::Vector3d current_vel_ = Eigen::Vector3d::Zero();
    //! 積分誤差
    Eigen::Vector3d integral_error_ = Eigen::Vector3d::Zero();

public:
    /**
     * @brief Construct a new Velocity Control object
     *
     * @param options
     */
    VelocityControl(const rclcpp::NodeOptions& options) : VelocityControl("", options) {}
    /**
     * @brief Construct a new Velocity Control object
     *
     * @param name_space
     * @param options
     */
    VelocityControl(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("velocity_control_node", name_space, options)
    {
        using namespace std::chrono_literals;

        declare_parameter("dt", 0.01);
        static double dt = get_parameter("dt").as_double();

        declare_parameter("xy_kp", 0.5);
        static double xy_kp = get_parameter("xy_kp").as_double();
        declare_parameter("xy_ki", 0.0);
        static double xy_ki = get_parameter("xy_ki").as_double();

        declare_parameter("yaw_kp", 0.5);
        static double yaw_kp = get_parameter("yaw_kp").as_double();
        declare_parameter("yaw_ki", 0.0);
        static double yaw_ki = get_parameter("yaw_ki").as_double();

        static auto cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 5);

        static auto target_vel_sub = create_subscription<geometry_msgs::msg::Twist>("target_vel", 5, [&](const geometry_msgs::msg::Twist::SharedPtr msg) { target_vel_ = make_eigen_vector3d(*msg); });
        static auto current_vel_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1, [&](nav_msgs::msg::Odometry::SharedPtr msg) { current_vel_ = make_eigen_vector3d(msg->twist.twist); });

        static auto timer = create_wall_timer(1s * dt, [&]() {
            if (target_vel_.norm() < 0.01) {
                integral_error_ = Eigen::Vector3d::Zero();
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel = make_twist(target_vel_);
                cmd_vel_pub->publish(cmd_vel);
                return;
            }
            auto vel = target_vel_;
            auto error = target_vel_ - current_vel_;
            integral_error_ += error * dt;
            vel.head<2>() += error.head<2>() * xy_kp;
            vel.head<2>() += integral_error_.head<2>() * xy_ki;
            vel.z() += error.z() * yaw_kp;
            vel.z() += integral_error_.z() * yaw_ki;
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel = make_twist(vel);
            cmd_vel_pub->publish(cmd_vel);
        });
    }
};

} // namespace tlab