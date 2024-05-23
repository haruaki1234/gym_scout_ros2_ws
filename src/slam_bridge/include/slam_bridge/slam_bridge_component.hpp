/**
 * @file slam_bridge_component.hpp
 * @author Takuma Nakao
 * @brief SLAMブリッジコンポーネント
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <chrono>
#include <algorithm>
#include <scout_msgs/msg/scout_status.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace tlab
{

class SlamBridge : public rclcpp::Node {
private:
    tf2_ros::TransformBroadcaster broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    nav_msgs::msg::Odometry last_odom_;

    std::ofstream pose_log_file_;
    std::optional<int64_t> first_time_ = std::nullopt;

    std::ofstream umap_log_file_;

    double odom_x_ = 0;
    double odom_y_ = 0;
    double odom_yaw_ = 0;

public:
    SlamBridge(const rclcpp::NodeOptions& options) : SlamBridge("", options) {}
    SlamBridge(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
        Node("slam_bridge_node", name_space, options), //
        broadcaster_(this),                            //
        tf_buffer_(this->get_clock()),                 //
        tf_listener_(tf_buffer_),                      //
        pose_log_file_("pose_log.csv"), umap_log_file_("umap_log.csv")
    {
        using namespace std::chrono_literals;
        tf_buffer_.setUsingDedicatedThread(true);

        declare_parameter("start_pos", std::vector<double>{0, 0, 0});
        static auto initial_pos = get_parameter("start_pos").as_double_array();

        static auto scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("slam_scan", 1);
        static auto error_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("error_marker", 1);

        static auto scout_status_sub = create_subscription<scout_msgs::msg::ScoutStatus>("/scout_status", 10, [&](const scout_msgs::msg::ScoutStatus::SharedPtr msg) {
            if (msg->control_mode == 1) {
                if (!first_time_) {
                    first_time_ = this->now().nanoseconds();
                }
            }
            else {
                first_time_ = std::nullopt;
            }
        });
        static auto odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 1, [&](nav_msgs::msg::Odometry::SharedPtr msg) {
            double init_x = initial_pos[0];
            double init_y = initial_pos[1];
            double init_yaw = initial_pos[2];
            odom_x_ = msg->pose.pose.position.x * std::cos(init_yaw) - msg->pose.pose.position.y * std::sin(init_yaw) + init_x;
            odom_y_ = msg->pose.pose.position.x * std::sin(init_yaw) + msg->pose.pose.position.y * std::cos(init_yaw) + init_y;
            odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation) + init_yaw;
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = msg->header.stamp;
            transform_stamped.header.frame_id = "slam_odom";
            transform_stamped.child_frame_id = "slam_base_link";
            transform_stamped.transform.translation.x = odom_x_;
            transform_stamped.transform.translation.y = odom_y_;
            transform_stamped.transform.translation.z = msg->pose.pose.position.z;
            transform_stamped.transform.rotation.z = std::sin(odom_yaw_ / 2);
            transform_stamped.transform.rotation.w = std::cos(odom_yaw_ / 2);
            broadcaster_.sendTransform(transform_stamped);
        });

        static auto ekf_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("ekf_odom", 1, [&](nav_msgs::msg::Odometry::SharedPtr msg) {
            geometry_msgs::msg::TransformStamped tf_map_to_slam_base_link;
            try {
                tf_map_to_slam_base_link = tf_buffer_.lookupTransform("map", "slam_base_link", rclcpp::Time(0));
            }
            catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
                return;
            }
            if (first_time_) {
                pose_log_file_ << (this->now().nanoseconds() - first_time_.value()) / 1e9 << ",";
                pose_log_file_ << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "," << tf2::getYaw(msg->pose.pose.orientation) << ",";
                pose_log_file_ << tf_map_to_slam_base_link.transform.translation.x << "," << tf_map_to_slam_base_link.transform.translation.y << "," << tf2::getYaw(tf_map_to_slam_base_link.transform.rotation) << ",";
                pose_log_file_ << odom_x_ << "," << odom_y_ << "," << odom_yaw_ << ",";
                pose_log_file_ << std::endl;
            }
        });

        static auto scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 1, [&](sensor_msgs::msg::LaserScan::SharedPtr msg) {
            sensor_msgs::msg::LaserScan scan_msg = *msg;
            scan_msg.header.frame_id = "slam_laser";
            scan_pub->publish(scan_msg);
        });

        static auto umap_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("umap_pos", 1, [&](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            geometry_msgs::msg::TransformStamped tf_map_to_slam_base_link;
            try {
                tf_map_to_slam_base_link = tf_buffer_.lookupTransform("map", "slam_base_link", msg->header.stamp);
            }
            catch (tf2::TransformException& ex) {
                // RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
                try {
                    tf_map_to_slam_base_link = tf_buffer_.lookupTransform("map", "slam_base_link", rclcpp::Time(0));
                }
                catch (tf2::TransformException& ex) {
                    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
                    return;
                }
            }
            umap_log_file_ << tf_map_to_slam_base_link.transform.translation.x << "," << tf_map_to_slam_base_link.transform.translation.y << "," << tf2::getYaw(tf_map_to_slam_base_link.transform.rotation) << ",";
            umap_log_file_ << msg->pose.pose.position.x << "," << msg->pose.pose.position.y << "," << tf2::getYaw(msg->pose.pose.orientation) << ",";
            umap_log_file_ << std::endl;
        });

        // static visualization_msgs::msg::MarkerArray marker_array;
        // static size_t marker_id = 0;
        // static auto timer = this->create_wall_timer(500ms, [&]() {
        //     geometry_msgs::msg::TransformStamped tf_base_link_to_slam_base_link;
        //     try {
        //         tf_base_link_to_slam_base_link = this->tf_buffer_.lookupTransform("base_link", "slam_base_link", rclcpp::Time(0));
        //     }
        //     catch (tf2::TransformException& ex) {
        //         RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        //         return;
        //     }
        //     geometry_msgs::msg::TransformStamped tf_map_to_base_link;
        //     try {
        //         tf_map_to_base_link = this->tf_buffer_.lookupTransform("map", "base_link", rclcpp::Time(0));
        //     }
        //     catch (tf2::TransformException& ex) {
        //         RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        //         return;
        //     }
        //     visualization_msgs::msg::Marker marker;
        //     marker.header.frame_id = "map";
        //     marker.header.stamp = this->now();
        //     marker.ns = "error";
        //     marker.id = marker_id++;
        //     marker.type = visualization_msgs::msg::Marker::SPHERE;
        //     marker.action = visualization_msgs::msg::Marker::ADD;
        //     marker.pose.position.x = tf_map_to_base_link.transform.translation.x;
        //     marker.pose.position.y = tf_map_to_base_link.transform.translation.y;
        //     marker.pose.position.z = tf_map_to_base_link.transform.translation.z;
        //     marker.pose.orientation.x = 0;
        //     marker.pose.orientation.y = 0;
        //     marker.pose.orientation.z = 0;
        //     marker.pose.orientation.w = 1;
        //     marker.scale.x = 0.1;
        //     marker.scale.y = 0.1;
        //     marker.scale.z = 0.1;
        //     double error = std::hypot(tf_base_link_to_slam_base_link.transform.translation.x, tf_base_link_to_slam_base_link.transform.translation.y);
        //     marker.color.a = 1.0;
        //     marker.color.r = std::min(error, 1.0);
        //     marker.color.g = std::max(1.0 - error, 0.0);
        //     marker.color.b = 0.0;
        //     marker_array.markers.push_back(marker);
        //     error_marker_pub->publish(marker_array);
        // });
    }
};

} // namespace tlab