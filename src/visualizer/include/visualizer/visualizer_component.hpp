#pragma once

#include <rclcpp/rclcpp.hpp>

namespace tlab
{

class Visualizer : public rclcpp::Node {
public:
    Visualizer(const rclcpp::NodeOptions& options) : Visualizer("", options) {}
    Visualizer(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("visualizer_node", name_space, options) { using namespace std::chrono_literals; }
};

} // namespace tlab