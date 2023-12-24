#include <rclcpp/rclcpp.hpp>
#include "route_following/route_following_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::RouteFollowing>());
    rclcpp::shutdown();
    return 0;
}