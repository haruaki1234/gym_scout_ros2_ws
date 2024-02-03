#include <rclcpp/rclcpp.hpp>
#include "velocity_control/velocity_control_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::VelocityControl>());
    rclcpp::shutdown();
    return 0;
}