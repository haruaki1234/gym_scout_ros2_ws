#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>

#include "utils/utils.hpp"
#include "utils/velocity_limit_filter.hpp"

namespace tlab
{

class Simulator : public rclcpp::Node {
private:
    Eigen::Vector3d current_pos_;
    Eigen::Vector3d current_vel_;
    double dt_ = 0.01;
    double max_vel_ = 1.0;
    double max_acc_ = 1.0;
    double max_ang_vel_ = 1.0;
    double max_ang_acc_ = 1.0;

    Eigen::Vector3d target_vel_;

public:
    Simulator(const rclcpp::NodeOptions& options) : Simulator("", options) {}
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
        current_pos_ = Eigen::Vector3d(get_parameter("init_x").as_double(), get_parameter("init_y").as_double(), get_parameter("init_yaw").as_double());
        current_vel_ = Eigen::Vector3d::Zero();

        target_vel_ = Eigen::Vector3d::Zero();

        static ClampVelLimitFilter vx_filter(-max_vel_, max_vel_, max_acc_, dt_);
        static ClampVelLimitFilter vy_filter(-max_vel_, max_vel_, max_acc_, dt_);
        static ClampVelLimitFilter vth_filter(-max_ang_vel_, max_ang_vel_, max_ang_acc_, dt_);
        vx_filter.reset(0);
        vy_filter.reset(0);
        vth_filter.reset(0);

        static auto current_vel_pub = create_publisher<geometry_msgs::msg::TwistStamped>("current_vel", rclcpp::QoS(10).reliable());
        static auto current_pos_pub = create_publisher<geometry_msgs::msg::PoseStamped>("current_pos", rclcpp::QoS(10).reliable());

        static auto target_vel_sub = create_subscription<geometry_msgs::msg::TwistStamped>("target_vel", rclcpp::QoS(10).reliable(), [&](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
            vx_filter.set_input(msg->twist.linear.x);
            vy_filter.set_input(msg->twist.linear.y);
            vth_filter.set_input(msg->twist.angular.z);
        });

        static auto timer = create_wall_timer(1s * dt_, [&]() {
            {
                current_vel_.x() = vx_filter.filtering();
                current_vel_.y() = vy_filter.filtering();
                current_vel_.z() = vth_filter.filtering();
                auto rotate_vel = rotate_2d(current_vel_.head<2>(), -current_pos_.z());
                current_pos_.x() += rotate_vel.x() * dt_;
                current_pos_.y() += rotate_vel.y() * dt_;
                current_pos_.z() += current_vel_.z() * dt_;
            }
            {
                geometry_msgs::msg::TwistStamped current_vel_msg;
                current_vel_msg.header.frame_id = "map";
                current_vel_msg.header.stamp = this->get_clock()->now();
                auto rotate_vel = rotate_2d(current_vel_.head<2>(), -current_pos_.z());
                current_vel_msg.twist = make_twist(rotate_vel.x(), rotate_vel.y(), current_vel_.z());
                current_vel_pub->publish(current_vel_msg);
            }
            {
                geometry_msgs::msg::PoseStamped current_pos_msg;
                current_pos_msg.header.frame_id = "map";
                current_pos_msg.header.stamp = this->get_clock()->now();
                current_pos_msg.pose = make_pose(current_pos_);
                current_pos_pub->publish(current_pos_msg);
            }
        });
    }
};

} // namespace tlab