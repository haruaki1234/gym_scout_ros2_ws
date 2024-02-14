#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <chrono>
#include <algorithm>
#include <scout_msgs/msg/scout_status.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace tlab
{

    class SlamBridge : public rclcpp::Node
    {
    private:
        tf2_ros::TransformBroadcaster broadcaster_;

    public:
        SlamBridge(const rclcpp::NodeOptions &options) : SlamBridge("", options) {}
        SlamBridge(const std::string &name_space = "", const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("slam_bridge_node", name_space, options), //
                                                                                                                     broadcaster_(this)                             //
        {
            using namespace std::chrono_literals;

            static nav_msgs::msg::Odometry::SharedPtr init_ekf_odom_msg;
            static nav_msgs::msg::Odometry::SharedPtr ekf_odom_msg;

            declare_parameter("start_pos", std::vector<double>{0, 0, 0});
            static auto initial_pos = get_parameter("start_pos").as_double_array();

            static auto scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("slam_scan", 1);

            static auto odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1, [&](nav_msgs::msg::Odometry::SharedPtr msg)
                                                                                      {
            double init_x = initial_pos[0];
            double init_y = initial_pos[1];
            double init_yaw = initial_pos[2];
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = msg->header.stamp;
            transform_stamped.header.frame_id = "slam_odom";
            transform_stamped.child_frame_id = "slam_base_link";
            transform_stamped.transform.translation.x = msg->pose.pose.position.x * std::cos(init_yaw) - msg->pose.pose.position.y * std::sin(init_yaw) + init_x;
            transform_stamped.transform.translation.y = msg->pose.pose.position.x * std::sin(init_yaw) + msg->pose.pose.position.y * std::cos(init_yaw) + init_y;
            transform_stamped.transform.translation.z = msg->pose.pose.position.z;
            double yaw = tf2::getYaw(msg->pose.pose.orientation) + init_yaw;
            transform_stamped.transform.rotation.z = std::sin(yaw / 2);
            transform_stamped.transform.rotation.w = std::cos(yaw / 2);
            broadcaster_.sendTransform(transform_stamped); });

            static auto scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, [&](sensor_msgs::msg::LaserScan::SharedPtr msg)
                                                                                          {
            sensor_msgs::msg::LaserScan scan_msg = *msg;
            scan_msg.header.frame_id = "slam_laser";
            scan_pub->publish(scan_msg); });
        }
    };

} // namespace tlab