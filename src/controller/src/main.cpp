#include <rclcpp/rclcpp.hpp>
#include "controller/controller_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::Controller>());
    rclcpp::shutdown();
    return 0;
}