#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>

#include "utils/utils.hpp"

#include "umap_image_sender.hpp"

namespace tlab
{

class UmapClient : public rclcpp::Node {
private:
    sensor_msgs::msg::Image::SharedPtr image_msg_;
    Eigen::Vector3d current_vel_;
    Eigen::Vector3d current_pos_;

    Eigen::Vector3d take_picture_pos_;

public:
    UmapClient(const rclcpp::NodeOptions& options) : UmapClient("", options) {}
    UmapClient(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("umap_client_node", name_space, options)
    {
        using namespace std::chrono_literals;

        declare_parameter("period", 0.01);
        static double period = get_parameter("period").as_double();

        declare_parameter("server_ip", "192.168.11.8");
        declare_parameter("server_port", 50000);
        declare_parameter("device_id", "TB01");
        declare_parameter("camera_number", 1);

        declare_parameter("resize_width", 320);
        declare_parameter("resize_height", 240);
        static auto resize_size = cv::Size(get_parameter("resize_width").as_int(), get_parameter("resize_height").as_int());

        declare_parameter("canny_threshould1", 50.0);
        static double canny_threshould1 = get_parameter("canny_threshould1").as_double();
        declare_parameter("canny_threshould2", 110.0);
        static double canny_threshould2 = get_parameter("canny_threshould2").as_double();

        declare_parameter("umap_matching_distance", -1.0);
        static double umap_matching_distance = get_parameter("umap_matching_distance").as_double();

        static umap::UmapImageSender umap_image_sender(get_parameter("server_ip").as_string(), get_parameter("server_port").as_int(), get_parameter("device_id").as_string(), get_parameter("camera_number").as_int());

        static auto umap_pos_pub = create_publisher<geometry_msgs::msg::PoseStamped>("umap_pos", rclcpp::QoS(10).reliable());

        static auto image_sub = this->create_subscription<sensor_msgs::msg::Image>("/image", rclcpp::QoS(rclcpp::KeepLast(3)), [&](sensor_msgs::msg::Image::SharedPtr msg) {
            image_msg_ = msg;
            take_picture_pos_ = current_pos_;
        });
        static auto localization_sub = create_subscription<nav_msgs::msg::Odometry>("localization", rclcpp::QoS(10).reliable(), [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_pos_ = make_eigen_vector3d(msg->pose.pose);
            current_vel_ = make_eigen_vector3d(msg->twist.twist);
        });

        static auto umap_shot_timer = create_wall_timer(1s * period, [&]() {
            if (!image_msg_) {
                return;
            }
            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(image_msg_, image_msg_->encoding);
            }
            catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            auto take_picture_pos = take_picture_pos_;

            cv::Mat resize_img;
            cv::resize(cv_ptr->image, resize_img, resize_size, 0, 0, cv::INTER_AREA);
            cv::Mat canny_img;
            cv::Canny(resize_img, canny_img, canny_threshould1, canny_threshould2);
            auto umap_result = umap_image_sender.send_from_mat(canny_img, umap::UmapImageSender::pose_t(take_picture_pos.x(), take_picture_pos.y(), 0, 0, 0, take_picture_pos.z()), umap_matching_distance);
            if (umap_result) {
                auto umap_pos = Eigen::Vector3d(umap_result->pose.x, umap_result->pose.y, umap_result->pose.yaw);
                auto localization_diff = current_pos_ - take_picture_pos;
                umap_pos.head<2>() += rotate_2d(localization_diff.head<2>(), umap_pos.z() - take_picture_pos.z());
                umap_pos.z() += localization_diff.z();

                geometry_msgs::msg::PoseStamped umap_pos_msg;
                umap_pos_msg.header.frame_id = "map";
                umap_pos_msg.header.stamp = this->get_clock()->now();
                umap_pos_msg.pose = make_pose(umap_pos);
                umap_pos_pub->publish(umap_pos_msg);
            }
        });
    }
};

} // namespace tlab