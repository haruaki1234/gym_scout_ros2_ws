#pragma once

#include <optional>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "utils/utils.hpp"
#include "a_star.hpp"

namespace tlab
{

class GlobalPathPlanner : public rclcpp::Node {
private:
    AStar a_star_;
    double resolution_ = 0.1;
    std::unordered_set<Eigen::Vector2i> map_;
    Eigen::Vector3d current_pos_;

    nav_msgs::msg::GridCells grid_msg_;

public:
    GlobalPathPlanner(const rclcpp::NodeOptions& options) : GlobalPathPlanner("", options) {}
    GlobalPathPlanner(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("global_path_planner_node", name_space, options)
    {
        using namespace std::chrono_literals;

        declare_parameter<std::string>("map_path", "");
        std::string map_path = get_parameter("map_path").as_string();

        declare_parameter<double>("min_x", 0.0);
        declare_parameter<double>("max_x", 0.0);
        declare_parameter<double>("min_y", 0.0);
        declare_parameter<double>("max_y", 0.0);
        double min_x = get_parameter("min_x").as_double();
        double max_x = get_parameter("max_x").as_double();
        double min_y = get_parameter("min_y").as_double();
        double max_y = get_parameter("max_y").as_double();

        declare_parameter<bool>("diagonal_movement", false);
        a_star_.set_diagonal_movement(get_parameter("diagonal_movement").as_bool());

        declare_parameter<int>("wall_search_range", 3);
        a_star_.set_wall_search_range(get_parameter("wall_search_range").as_int());

        declare_parameter<double>("wall_cost_waight", 1.0);
        a_star_.set_wall_cost_waight(get_parameter("wall_cost_waight").as_double());

        cv::Mat cv_map = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
        if (cv_map.empty()) {
            RCLCPP_ERROR(this->get_logger(), "failed to load map");
            return;
        }
        resolution_ = std::max((max_x - min_x) / cv_map.cols, (max_y - min_y) / cv_map.rows);
        for (int y = 0; y < cv_map.rows; y++) {
            for (int x = 0; x < cv_map.cols; x++) {
                if (cv_map.at<uchar>(y, x) == 0) {
                    auto pos = Eigen::Vector2d((max_x - min_x) / cv_map.cols * x + min_x, (max_y - min_y) / cv_map.rows * y + min_y);
                    pos.y() = max_y + min_y - pos.y();
                    map_.insert(to_grid(pos));
                    geometry_msgs::msg::Point point;
                    point.x = to_world(to_grid(pos)).x();
                    point.y = to_world(to_grid(pos)).y();
                    grid_msg_.cells.push_back(point);
                }
            }
        }

        static auto path_pub = this->create_publisher<nav_msgs::msg::Path>("global_path", rclcpp::QoS(10));
        static auto grid_pub = this->create_publisher<nav_msgs::msg::GridCells>("map_grid", rclcpp::QoS(10));
        grid_msg_.header.frame_id = "map";
        grid_msg_.header.stamp = this->get_clock()->now();
        grid_msg_.cell_width = resolution_;
        grid_msg_.cell_height = resolution_;
        grid_pub->publish(grid_msg_);

        static auto current_pos_sub = create_subscription<geometry_msgs::msg::PoseStamped>("current_pos", rclcpp::QoS(10).reliable(), [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { current_pos_ = make_eigen_vector3d(msg->pose); });

        static auto goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", rclcpp::QoS(10).reliable(), [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            auto goal = to_grid(Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y));
            if (map_.find(goal) != map_.end()) {
                RCLCPP_WARN(this->get_logger(), "goal position is in the wall");
                return;
            }
            auto path = a_star_.find_path(to_grid(current_pos_.head<2>()), goal, map_);
            RCLCPP_INFO(this->get_logger(), "path found");
            nav_msgs::msg::Path path_msg;
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = this->get_clock()->now();
            for (int i = 0; i < (int)path.size(); i++) {
                auto world_pos = to_world(path[i]);
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_msg.header;
                pose.header.stamp = this->get_clock()->now();
                int i1 = std::max(i - 1, 0);
                int i2 = std::min(i + 1, (int)path.size() - 1);
                double yaw = std::atan2(path[i2].y() - path[i1].y(), path[i2].x() - path[i1].x());
                pose.pose = make_pose(world_pos.x(), world_pos.y(), yaw);
                path_msg.poses.push_back(pose);
            }
            path_pub->publish(path_msg);
        });

        static auto timer = this->create_wall_timer(100ms, [&]() { grid_pub->publish(grid_msg_); });
    }

    Eigen::Vector2i to_grid(const Eigen::Vector2d& pos) const { return Eigen::Vector2i(pos.x() / resolution_, pos.y() / resolution_); }
    Eigen::Vector2d to_world(const Eigen::Vector2i& pos) const { return Eigen::Vector2d((pos.x() + 0.5) * resolution_, (pos.y() + 0.5) * resolution_); }
};

} // namespace tlab