/**
 * @file visualizer_component.hpp
 * @author Takuma Nakao
 * @brief ビジュアライザーコンポーネント
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace tlab
{

class Visualizer : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Visualizer object
     *
     * @param options
     */
    Visualizer(const rclcpp::NodeOptions& options) : Visualizer("", options) {}
    /**
     * @brief Construct a new Visualizer object
     *
     * @param name_space
     * @param options
     */
    Visualizer(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("visualizer_node", name_space, options)
    {
        using namespace std::chrono_literals;
        static auto publisher = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        static auto ekf_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("ekf_odom", 10, [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "my_namespace";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = msg->pose.pose.position.x;
            marker.pose.position.y = msg->pose.pose.position.y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 2.0;
            marker.scale.y = 2.0;
            marker.scale.z = 2.0;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            publisher->publish(marker);
        });
    }
};

} // namespace tlab