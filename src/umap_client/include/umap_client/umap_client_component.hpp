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

        static umap::UmapImageSender umap_image_sender("192.168.11.1", 8080, "TB01", 1);

        static auto timer = create_wall_timer(1s * period_, [&]() { RCLCPP_INFO(this->get_logger(), "Hello, world!"); });
    }
};

} // namespace tlab