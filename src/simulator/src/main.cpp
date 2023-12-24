#include <rclcpp/rclcpp.hpp>
#include "simulator/simulator_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::Simulator>());
    rclcpp::shutdown();
    return 0;
}