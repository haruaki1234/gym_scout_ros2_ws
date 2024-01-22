#pragma once

#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>

#include "utils/utils.hpp"
#include "utils/math_util.hpp"
#include "utils/golden_search.hpp"
#include "utils/kd_tree.hpp"

namespace tlab
{

class RouteFollowing : public rclcpp::Node {
private:
    struct local_path_data_t {
        Eigen::Vector3d vel;
        std::vector<Eigen::Vector3d> path;
        local_path_data_t(const Eigen::Vector3d& _vel, const std::vector<Eigen::Vector3d>& _path) : vel(_vel), path(_path) {}
    };

    class Score {
    private:
        double weight_;
        std::function<double(const local_path_data_t&)> func_;
        std::vector<double> normalize(std::vector<double> data) const
        {
            double max = -std::numeric_limits<double>::infinity();
            double min = std::numeric_limits<double>::infinity();
            for (const auto& v : data) {
                if (std::isfinite(v) && v > max) {
                    max = v;
                }
                if (std::isfinite(v) && v < min) {
                    min = v;
                }
            }

            for (auto& v : data) {
                if (!std::isfinite(max) || !std::isfinite(min)) {
                    v = 0;
                }
                else if (std::abs(max - min) < 1e-6) {
                    v = 0;
                }
                else if (std::isfinite(v)) {
                    v = (v - min) / (max - min);
                }
            }
            return data;
        }

    public:
        Score(double weight, std::function<double(const local_path_data_t&)> func) : weight_(weight), func_(func) {}
        Eigen::VectorXd operator()(const std::vector<local_path_data_t>& path) const
        {
            std::vector<double> data;
            data.reserve(path.size());
            for (const auto& p : path) {
                data.push_back(func_(p));
            }
            data = normalize(data);
            Eigen::VectorXd score = Eigen::VectorXd::Zero(path.size());
            for (size_t i = 0; i < path.size(); i++) {
                score(i) = data[i] * weight_;
            }
            return score;
        }
    };

    Eigen::Vector3d current_vel_;
    Eigen::Vector3d current_pos_;

    std::vector<geometry_msgs::msg::PoseStamped> global_path_;
    Eigen::Vector3d local_target_pos_;
    std::optional<kd_tree::EigenVectorTree<2>> map_grid_;

    int near_index_ = 0;

    double control_period_;
    double max_vel_;
    double max_acc_;
    double max_angle_vel_;
    double max_angle_acc_;
    int vel_split_num_;
    double simulation_time_;
    int simulation_step_;
    double collision_map_distance_;
    double ignore_map_distance_;

    double score_local_target_distance_waight_;
    double score_local_target_angle_waight_;
    double score_map_distance_waight_;
    double score_integral_error_waight_;

    double near_goal_control_distance_;
    double near_goal_control_pos_kp_;
    double near_goal_control_angle_kp_;

    double goal_distance_;

    std::vector<Score> scores_;

public:
    RouteFollowing(const rclcpp::NodeOptions& options) : RouteFollowing("", options) {}
    RouteFollowing(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("route_following_node", name_space, options)
    {
        using namespace std::chrono_literals;

        declare_parameter<double>("control_period", 0.1);
        control_period_ = get_parameter("control_period").as_double();
        declare_parameter<double>("max_vel", 1.0);
        max_vel_ = get_parameter("max_vel").as_double();
        declare_parameter<double>("max_acc", 1.0);
        max_acc_ = get_parameter("max_acc").as_double();
        declare_parameter<double>("max_angle_vel", 1.0);
        max_angle_vel_ = get_parameter("max_angle_vel").as_double();
        declare_parameter<double>("max_angle_acc", 1.0);
        max_angle_acc_ = get_parameter("max_angle_acc").as_double();
        declare_parameter<int>("vel_split_num", 10);
        vel_split_num_ = get_parameter("vel_split_num").as_int();
        declare_parameter<double>("simulation_time", 1.0);
        simulation_time_ = get_parameter("simulation_time").as_double();
        declare_parameter<int>("simulation_step", 10);
        simulation_step_ = get_parameter("simulation_step").as_int();
        declare_parameter<double>("collision_map_distance", 1.0);
        collision_map_distance_ = get_parameter("collision_map_distance").as_double();
        declare_parameter<double>("ignore_map_distance", 1.0);
        ignore_map_distance_ = get_parameter("ignore_map_distance").as_double();

        declare_parameter<double>("score_local_target_distance_waight", 1.0);
        score_local_target_distance_waight_ = get_parameter("score_local_target_distance_waight").as_double();
        declare_parameter<double>("score_local_target_angle_waight", 1.0);
        score_local_target_angle_waight_ = get_parameter("score_local_target_angle_waight").as_double();
        declare_parameter<double>("score_map_distance_waight", 1.0);
        score_map_distance_waight_ = get_parameter("score_map_distance_waight").as_double();
        declare_parameter<double>("score_integral_error_waight", 1.0);
        score_integral_error_waight_ = get_parameter("score_integral_error_waight").as_double();

        declare_parameter<double>("near_goal_control_distance", 1.0);
        near_goal_control_distance_ = get_parameter("near_goal_control_distance").as_double();
        declare_parameter<double>("near_goal_control_pos_kp", 1.0);
        near_goal_control_pos_kp_ = get_parameter("near_goal_control_pos_kp").as_double();
        declare_parameter<double>("near_goal_control_angle_kp", 1.0);
        near_goal_control_angle_kp_ = get_parameter("near_goal_control_angle_kp").as_double();

        declare_parameter<double>("goal_distance", 1.0);
        goal_distance_ = get_parameter("goal_distance").as_double();

        scores_.push_back(Score(score_local_target_distance_waight_, [&](const local_path_data_t& path) { return calc_score_local_target_distance(path); }));
        scores_.push_back(Score(score_local_target_angle_waight_, [&](const local_path_data_t& path) { return calc_score_local_target_angle(path); }));
        scores_.push_back(Score(score_map_distance_waight_, [&](const local_path_data_t& path) { return calc_score_map_distance(path); }));
        scores_.push_back(Score(score_integral_error_waight_, [&](const local_path_data_t& path) { return calc_score_integral_error(path); }));

        static auto velocity_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 5);
        static auto candidate_path_pub = create_publisher<nav_msgs::msg::Path>("local_path_candidate", rclcpp::QoS(10).reliable());
        static auto local_path_pub = create_publisher<nav_msgs::msg::Path>("local_path", rclcpp::QoS(10).reliable());
        static auto local_target_pos_pub = create_publisher<geometry_msgs::msg::PoseStamped>("local_target_pos", rclcpp::QoS(10).reliable());

        static auto localization_sub = create_subscription<nav_msgs::msg::Odometry>("localization", rclcpp::QoS(10).reliable(), [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_pos_ = make_eigen_vector3d(msg->pose.pose);
            current_vel_ = make_eigen_vector3d(msg->twist.twist);
        });
        static auto global_path_sub = create_subscription<nav_msgs::msg::Path>("global_path", rclcpp::QoS(10).reliable(), [&](const nav_msgs::msg::Path::SharedPtr msg) {
            global_path_ = msg->poses;
            near_index_ = 0;
        });
        static auto map_grid_sub = create_subscription<nav_msgs::msg::GridCells>("map_grid", rclcpp::QoS(10).reliable(), [&](const nav_msgs::msg::GridCells::SharedPtr msg) {
            static auto stamp = this->get_clock()->now();
            if (stamp != rclcpp::Time(msg->header.stamp)) {
                std::vector<Eigen::Vector2d> map_grid;
                map_grid.reserve(msg->cells.size());
                for (const auto& c : msg->cells) {
                    map_grid.push_back(Eigen::Vector2d(c.x, c.y));
                }
                map_grid_ = kd_tree::EigenVectorTree<2>::build(map_grid);
                stamp = msg->header.stamp;
                RCLCPP_INFO(this->get_logger(), "map_grid update");
            }
        });

        static auto path_publish = [&](const std::vector<Eigen::Vector3d>& path, const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub) {
            nav_msgs::msg::Path path_msg;
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = this->get_clock()->now();
            for (const auto& p : path) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = p.x();
                pose.pose.position.y = p.y();
                path_msg.poses.push_back(pose);
            }
            pub->publish(path_msg);
        };

        static auto timer = create_wall_timer(1s * control_period_, [&]() {
            if (global_path_.empty()) {
                geometry_msgs::msg::Twist vel_msg;
                vel_msg.linear.x = 0;
                vel_msg.linear.y = 0;
                vel_msg.angular.z = 0;
                velocity_pub->publish(vel_msg);
                return;
            }

            {
                double min_e = std::numeric_limits<double>::infinity();
                for (size_t i = near_index_; i < global_path_.size(); i++) {
                    auto pos = Eigen::Vector2d(global_path_[i].pose.position.x, global_path_[i].pose.position.y);
                    double e = (pos - current_pos_.head<2>()).norm();
                    if (e < min_e) {
                        min_e = e;
                        near_index_ = i;
                    }
                }
                int target_index = near_index_;
                double route_distance = 0;
                auto prev_pos = Eigen::Vector2d(global_path_[near_index_].pose.position.x, global_path_[near_index_].pose.position.y);
                for (size_t i = target_index; i < global_path_.size(); i++) {
                    auto pos = Eigen::Vector2d(global_path_[i].pose.position.x, global_path_[i].pose.position.y);
                    route_distance += (pos - prev_pos).norm();
                    prev_pos = pos;
                    target_index = i;
                    if (route_distance >= max_vel_ * simulation_time_) {
                        break;
                    }
                }
                local_target_pos_ = make_eigen_vector3d(global_path_[target_index].pose);
                geometry_msgs::msg::PoseStamped local_target_pos_msg;
                local_target_pos_msg.header.frame_id = "map";
                local_target_pos_msg.header.stamp = this->get_clock()->now();
                local_target_pos_msg.pose = make_pose(local_target_pos_);
                local_target_pos_pub->publish(local_target_pos_msg);
            }

            if (auto goal_pos = make_eigen_vector3d(global_path_.back().pose); (goal_pos - current_pos_).head<2>().norm() < goal_distance_) {
                global_path_.clear();
                geometry_msgs::msg::Twist vel_msg;
                vel_msg.linear.x = 0;
                vel_msg.linear.y = 0;
                vel_msg.angular.z = 0;
                velocity_pub->publish(vel_msg);
                return;
            }
            else if ((goal_pos - current_pos_).head<2>().norm() < near_goal_control_distance_) {
                auto e = goal_pos - current_pos_;
                auto target_vel = Eigen::Vector3d(e.x() * near_goal_control_pos_kp_, e.y() * near_goal_control_pos_kp_, e.z() * near_goal_control_angle_kp_);
                target_vel.head<2>() = rotate_2d(target_vel.head<2>(), -current_pos_.z());
                target_vel.x() = std::clamp(target_vel.x(), -max_vel_, max_vel_);
                target_vel.y() = std::clamp(target_vel.y(), -max_vel_, max_vel_);
                target_vel.z() = std::clamp(target_vel.z(), -max_angle_vel_, max_angle_vel_);
                geometry_msgs::msg::Twist vel_msg;
                vel_msg = make_twist(target_vel);
                velocity_pub->publish(vel_msg);
                return;
            }

            auto start_time = this->get_clock()->now();

            auto choices_path = make_choices_path();
            for (auto& p : choices_path) {
                path_publish(p.path, candidate_path_pub);
            }

            Eigen::VectorXd score = Eigen::VectorXd::Zero(choices_path.size());
            for (const auto& s : scores_) {
                score += s(choices_path);
            }
            auto itr = std::max_element(score.data(), score.data() + score.size());
            auto index = std::distance(score.data(), itr);
            auto target_vel = choices_path[index].vel;
            geometry_msgs::msg::Twist vel_msg;
            vel_msg = make_twist(target_vel);
            velocity_pub->publish(vel_msg);
            path_publish(choices_path[index].path, local_path_pub);
            if ((this->get_clock()->now() - start_time).seconds() > control_period_) {
                RCLCPP_WARN(this->get_logger(), "calc time: %f", (this->get_clock()->now() - start_time).seconds());
            }
        });
    }

private:
    std::vector<Eigen::Vector3d> simulation_pos(const Eigen::Vector3d& vel, Eigen::Vector3d pos) const
    {
        std::vector<Eigen::Vector3d> pos_vec;
        pos_vec.reserve(simulation_step_);
        double dt = simulation_time_ / simulation_step_;
        for (int i = 0; i < simulation_step_; i++) {
            auto rotate_vel = rotate_2d(vel.head<2>(), pos.z());
            pos.x() += rotate_vel.x() * dt;
            pos.y() += rotate_vel.y() * dt;
            pos.z() += vel.z() * dt;
            pos_vec.push_back(pos);
        }
        return pos_vec;
    }
    std::vector<local_path_data_t> make_choices_path() const
    {
        auto rotate_vel = rotate_2d(current_vel_.head<2>(), -current_pos_.z());
        double range_min_x = std::max(rotate_vel.x() - max_acc_ * control_period_, -max_vel_);
        double range_max_x = std::min(rotate_vel.x() + max_acc_ * control_period_, max_vel_);
        double range_min_y = std::max(rotate_vel.y() - max_acc_ * control_period_, -max_vel_);
        double range_max_y = std::min(rotate_vel.y() + max_acc_ * control_period_, max_vel_);
        double range_min_z = std::max(current_vel_.z() - max_angle_acc_ * control_period_, -max_angle_vel_);
        double range_max_z = std::min(current_vel_.z() + max_angle_acc_ * control_period_, max_angle_vel_);

        std::vector<local_path_data_t> choices_path;
        for (int i = 0; i <= vel_split_num_; i++) {
            double x = range_min_x + (range_max_x - range_min_x) / vel_split_num_ * i;
            for (int j = 0; j <= vel_split_num_; j++) {
                double y = range_min_y + (range_max_y - range_min_y) / vel_split_num_ * j;
                for (int k = 0; k <= vel_split_num_; k++) {
                    double z = range_min_z + (range_max_z - range_min_z) / vel_split_num_ * k;
                    choices_path.push_back(local_path_data_t(Eigen::Vector3d(x, y, z), simulation_pos(Eigen::Vector3d(x, y, z), current_pos_)));
                }
            }
        }
        return choices_path;
    }
    double calc_score_local_target_distance(const local_path_data_t& path) const { return -(local_target_pos_ - path.path.back()).head<2>().norm(); }
    double calc_score_local_target_angle(const local_path_data_t& path) const { return -std::abs(normalize_angle((local_target_pos_ - path.path.back()).z())); }
    double calc_score_map_distance(const local_path_data_t& path) const
    {
        if (!map_grid_) {
            return 0;
        }
        double score = 0;
        for (const auto& p : path.path) {
            auto pos = p.head<2>();
            double d = (pos - map_grid_->nn_serch(pos)).norm();
            if (d < collision_map_distance_) {
                return -std::numeric_limits<double>::infinity();
            }
            else if (d < ignore_map_distance_) {
                score -= 1.0 / d;
            }
        }
        return score;
    }
    double calc_score_integral_error(const local_path_data_t& path) const
    {
        double score = 0;
        double path_len = 0;
        int near = 0;
        Eigen::Vector2d prev_pos = path.path.front().head<2>();
        for (const auto& p : path.path) {
            auto distance = [&](const int n) {
                Eigen::Vector2d global_path_pos(global_path_[n].pose.position.x, global_path_[n].pose.position.y);
                return (global_path_pos - p.head<2>()).norm();
            };
            near = golden_search<int>(distance, near, global_path_.size() - 1, 10);
            Eigen::Vector2d global_path_pos(global_path_[near].pose.position.x, global_path_[near].pose.position.y);
            score -= (global_path_pos - p.head<2>()).norm();
            path_len += (prev_pos - p.head<2>()).norm();
            prev_pos = p.head<2>();
        }
        return score / path_len;
    }
};

} // namespace tlab