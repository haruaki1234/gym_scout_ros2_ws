#include <rclcpp/rclcpp.hpp>
#include "localization/localization_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::Localization>());
    rclcpp::shutdown();
    return 0;
}