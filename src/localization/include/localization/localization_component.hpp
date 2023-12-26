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
#include "utils/utils.hpp"
#include "utils/kalman_filter.hpp"

namespace tlab
{

class Localization : public rclcpp::Node {
private:
    tf2_ros::TransformBroadcaster broadcaster_;

    Eigen::Vector3d odometer_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d odometer_vel_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d estimate_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d estimate_vel_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d umap_pos_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d initial_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector2d reference_grid_point_ = Eigen::Vector2d::Zero();

    KalmanFilter<double, 3, 3, 3> klf_;
    rclcpp::Time position_reset_time_;

public:
    Localization(const rclcpp::NodeOptions& options) : Localization("", options) {}
    Localization(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("localization_node", name_space, options), broadcaster_(this)
    {
        using namespace std::chrono_literals;
        position_reset_time_ = this->get_clock()->now();

        declare_parameter("period", 0.01);
        static const double PERIOD = get_parameter("period").as_double();

        declare_parameter("start_pos", std::vector<double>{0, 0, 0});
        auto initial_pos = get_parameter("start_pos").as_double_array();

        declare_parameter("grid_point_observer.grid_width", 0.01);
        static const double GRID_WIDTH = get_parameter("grid_point_observer.grid_width").as_double();

        declare_parameter("grid_point_observer.gain_xy", 0.1);
        static const double GAIN_XY = get_parameter("grid_point_observer.gain_xy").as_double();

        declare_parameter("grid_point_observer.gain_theta", 0.03);
        static const double GAIN_THETA = get_parameter("grid_point_observer.gain_theta").as_double();

        static auto set_position = [&](const Eigen::Vector3d& init_pos) {
            position_reset_time_ = this->get_clock()->now();
            initial_pos_.head<2>() = init_pos.head<2>();
            initial_pos_.z() = 0;
            odometer_pos_ = init_pos;
            Eigen::Matrix<double, 3, 3> p0 = Eigen::Matrix<double, 3, 3>::Zero();
            klf_.reset(init_pos, p0);
            reference_grid_point_.x() = std::floor(init_pos.x() / GRID_WIDTH + 0.5) * GRID_WIDTH;
            reference_grid_point_.y() = std::floor(init_pos.y() / GRID_WIDTH + 0.5) * GRID_WIDTH;
        };

        static auto current_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("current_vel", rclcpp::QoS(10).reliable());
        static auto current_pos_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pos", rclcpp::QoS(10).reliable());

        static auto odometer_vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>("odom_vel", rclcpp::QoS(10).reliable(), [&](const geometry_msgs::msg::TwistStamped::SharedPtr msg) { odometer_vel_ = make_eigen_vector3d(msg->twist); });
        static auto odometer_pos_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("odom_pos", rclcpp::QoS(10).reliable(), [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { odometer_pos_ = make_eigen_vector3d(msg->pose); });

        set_position(Eigen::Vector3d(initial_pos[0], initial_pos[1], initial_pos[2]));

        klf_.F = Eigen::Vector3d::Ones().asDiagonal();
        Eigen::Vector3d g_vec;
        g_vec << PERIOD, PERIOD, PERIOD;
        klf_.G = g_vec.asDiagonal();
        klf_.H = Eigen::Vector3d::Ones().asDiagonal();

        Eigen::Vector3d q_vec;
        q_vec << 0.01, 0.01, 0.01;
        klf_.Q = q_vec.asDiagonal();

        Eigen::Vector3d r_vec;
        r_vec << 0.0001, 0.0001, 0.0001;
        klf_.R = r_vec.asDiagonal();

        static auto set_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", rclcpp::QoS(10).reliable(), [&](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) { set_position(make_eigen_vector3d(msg->pose.pose)); });

        static auto umap_pos = this->create_subscription<geometry_msgs::msg::PoseStamped>("umap_pos", rclcpp::QoS(10).reliable(), [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            if (this->get_clock()->now().seconds() - position_reset_time_.seconds() < 0.2) {
                return;
            }
            umap_pos_ = make_eigen_vector3d(msg->pose);
            umap_pos_ += estimate_vel_ * static_cast<double>(this->get_clock()->now().seconds() - rclcpp::Time(msg->header.stamp).seconds());

            // 格子点オブザーバ
            // 基準格子点更新
            auto old_grid_point = reference_grid_point_;
            reference_grid_point_.x() = std::floor(odometer_pos_.x() / GRID_WIDTH + 0.5) * GRID_WIDTH;
            reference_grid_point_.y() = std::floor(odometer_pos_.y() / GRID_WIDTH + 0.5) * GRID_WIDTH;
            auto initial_pos_xy = initial_pos_.head<2>() - rotate_2d(old_grid_point, initial_pos_.z()) + rotate_2d(reference_grid_point_, initial_pos_.z());
            initial_pos_.head<2>() = initial_pos_xy;

            Eigen::Vector2d q = odometer_pos_.head<2>() - reference_grid_point_;
            q = rotate_2d(q, initial_pos_.z());
            for (size_t i = 0; i < 2; i++) {
                initial_pos_[i] -= GAIN_XY * (q[i] + initial_pos_[i] - umap_pos_[i]);
            }
            initial_pos_[2] = normalize_angle(initial_pos_[2] - GAIN_THETA * normalize_angle(odometer_pos_[2] + initial_pos_[2] - umap_pos_[2]));
        });

        static auto timer = this->create_wall_timer(1s * PERIOD, [&]() {
            // 格子点オブザーバ
            // 基準格子点更新
            auto old_grid_point = reference_grid_point_;
            reference_grid_point_.x() = std::floor(odometer_pos_.x() / GRID_WIDTH + 0.5) * GRID_WIDTH;
            reference_grid_point_.y() = std::floor(odometer_pos_.y() / GRID_WIDTH + 0.5) * GRID_WIDTH;
            auto initial_pos_xy = initial_pos_.head<2>() - rotate_2d(old_grid_point, initial_pos_.z()) + rotate_2d(reference_grid_point_, initial_pos_.z());
            initial_pos_.head<2>() = initial_pos_xy;
            Eigen::Vector2d q = odometer_pos_.head<2>() - reference_grid_point_;
            q = rotate_2d(q, initial_pos_.z());
            for (int i = 0; i < 2; i++) {
                estimate_pos_[i] = initial_pos_[i] + q[i];
            }
            estimate_pos_[2] = normalize_angle(initial_pos_[2] + odometer_pos_[2]);
            estimate_vel_.head<2>() = rotate_2d(odometer_vel_.head<2>(), estimate_pos_.z());
            estimate_vel_.z() = odometer_vel_.z();

            estimate_pos_ = klf_.filtering(estimate_vel_, estimate_pos_);

            geometry_msgs::msg::TwistStamped current_vel_msg;
            current_vel_msg.header.frame_id = "map";
            current_vel_msg.header.stamp = this->get_clock()->now();
            current_vel_msg.twist = make_twist(estimate_vel_);
            current_vel_pub->publish(current_vel_msg);

            geometry_msgs::msg::PoseStamped current_pos_msg;
            current_pos_msg.header.frame_id = "map";
            current_pos_msg.header.stamp = this->get_clock()->now();
            current_pos_msg.pose = make_pose(estimate_pos_);
            current_pos_pub->publish(current_pos_msg);

            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = this->get_clock()->now();
            transform_stamped.header.frame_id = "map";
            transform_stamped.child_frame_id = "base_link";
            transform_stamped.transform = make_geometry_transform(estimate_pos_);

            broadcaster_.sendTransform(transform_stamped);
        });
    }
};

} // namespace tlab