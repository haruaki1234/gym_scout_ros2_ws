#include <rclcpp/rclcpp.hpp>
#include "slam_bridge/slam_bridge_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::SlamBridge>());
    rclcpp::shutdown();
    return 0;
}