#include <rclcpp/rclcpp.hpp>
#include "visualizer/visualizer_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::Visualizer>());
    rclcpp::shutdown();
    return 0;
}