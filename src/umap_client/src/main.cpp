#include <rclcpp/rclcpp.hpp>
#include "umap_client/umap_client_component.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tlab::UmapClient>());
    rclcpp::shutdown();
    return 0;
}