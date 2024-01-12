#pragma once

#include <rclcpp/rclcpp.hpp>

#include "umap_image_sender.hpp"

namespace tlab
{

class UmapClient : public rclcpp::Node {
private:
    double period_;

public:
    UmapClient(const rclcpp::NodeOptions& options) : UmapClient("", options) {}
    UmapClient(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("umap_client_node", name_space, options)
    {
        using namespace std::chrono_literals;

        declare_parameter("period", 0.01);
        period_ = get_parameter("period").as_double();

        declare_parameter("server_ip", "192.168.11.8");
        declare_parameter("server_port", 50000);
        declare_parameter("device_id", "TB01");
        declare_parameter("camera_number", 1);

        static umap::UmapImageSender umap_image_sender(get_parameter("server_ip").as_string(), get_parameter("server_port").as_int(), get_parameter("device_id").as_string(), get_parameter("camera_number").as_int());
        umap_image_sender.is_return = false;

        static auto timer = create_wall_timer(1s * period_, [&]() {
            cv::Mat read_img = cv::imread("src/bringup/config/IMG_1582.JPG");
            cv::Mat resize_img;
            cv::resize(read_img, resize_img, cv::Size(320, 240), 0, 0, cv::INTER_AREA);
            cv::Mat canny_img;
            cv::Canny(resize_img, canny_img, 50, 110);
            umap_image_sender.send_from_mat(canny_img, umap::UmapImageSender::pose_t(), -1);
        });
    }
};

} // namespace tlab