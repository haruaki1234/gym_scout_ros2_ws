#include <rclcpp/rclcpp.hpp>
#include "global_path_planner/global_path_planner_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::GlobalPathPlanner>());
    rclcpp::shutdown();
    return 0;
}