/**
 * @file controller_component.hpp
 * @author Takuma Nakao
 * @brief 目標位置に到達するたびに次の目標位置を設定するコンポーネント
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <random>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <scout_msgs/msg/scout_status.hpp>

#include "utils/utils.hpp"

namespace tlab
{

/**
 * @brief　コントローラノード
 *
 */
class Controller : public rclcpp::Node {
private:
    //! Scoutステータスメッセージ
    scout_msgs::msg::ScoutStatus::SharedPtr scout_status_msg_;
    //! 現在の状態
    int state_ = -1;
    //! 目標経由点のインデックス
    int target_index_ = 0;
    //! 経路追従モードの状態
    std::string route_follow_state_ = "wait_global_path";
    //! 待機開始時間
    rclcpp::Time wait_start_time_;

public:
    /**
     * @brief Construct a new Controller object
     *
     * @param options
     */
    Controller(const rclcpp::NodeOptions& options) : Controller("", options) {}
    /**
     * @brief Construct a new Controller object
     *
     * @param name_space
     * @param options
     */
    Controller(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("controller_node", name_space, options)
    {
        using namespace std::chrono_literals;

        static std::vector<Eigen::Vector3d> waypoints = {
            // Eigen::Vector3d(2.6, 2.0, 0.0),
            Eigen::Vector3d(34.0, 1.2, 0.0),
            // Eigen::Vector3d(2.6, 1.2, 0.0),
            Eigen::Vector3d(2.6, 42.0, 0.0),
        };

        // static std::vector<Eigen::Vector3d> waypoints = {
        //     Eigen::Vector3d(34.0, 1.2, 0.0),
        //     Eigen::Vector3d(6.3, 1.2, 0.0),
        // };

        static auto goal_pos_pub = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", rclcpp::QoS(10).reliable());

        static auto scout_status_sub = create_subscription<scout_msgs::msg::ScoutStatus>("/scout_status", 10, [&](const scout_msgs::msg::ScoutStatus::SharedPtr msg) { scout_status_msg_ = msg; });
        static auto route_follow_state_sub = create_subscription<std_msgs::msg::String>("route_follow_state", rclcpp::QoS(10).reliable(), [&](const std_msgs::msg::String::SharedPtr msg) { route_follow_state_ = msg->data; });

        static auto timer = create_wall_timer(1s * 0.1, [&]() {
            if (!scout_status_msg_) {
                return;
            }
            if (scout_status_msg_->control_mode == 1) {
                geometry_msgs::msg::PoseStamped goal_pose;
                switch (state_) {
                case -1:
                    state_ = 0;
                    break;
                case 0:
                    goal_pose.pose = make_pose(waypoints[target_index_]);
                    goal_pos_pub->publish(goal_pose);
                    target_index_ = (target_index_ + 1) % waypoints.size();
                    state_ = 1;
                    break;
                case 1:
                    if (route_follow_state_ != "wait_global_path") {
                        state_ = 2;
                    }
                    break;
                case 2:
                    if (route_follow_state_ == "wait_global_path" || route_follow_state_ == "goal") {
                        wait_start_time_ = this->get_clock()->now();
                        state_ = 3;
                    }
                    break;
                case 3:
                    if ((this->get_clock()->now() - wait_start_time_).seconds() > 1) {
                        state_ = 0;
                    }
                    break;
                }
            }
        });
    }
};

} // namespace tlab