/**
 * @file route_following_component.hpp
 * @author Takuma Nakao
 * @brief 局所的経路計画による経路追従
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
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
#include "utils/covariance_index.hpp"

namespace tlab
{

/**
 * @brief 経路追従ノード
 *
 */
class RouteFollowing : public rclcpp::Node {
private:
    /**
     * @brief 候補パスデータ
     *
     */
    struct local_path_data_t {
        //! 指示速度
        Eigen::Vector3d vel;
        //! パス
        std::vector<Eigen::Vector3d> path;
        /**
         * @brief Construct a new local path data t object
         *
         * @param _vel 指示速度
         * @param _path パス
         */
        local_path_data_t(const Eigen::Vector3d& _vel, const std::vector<Eigen::Vector3d>& _path) : vel(_vel), path(_path) {}
    };

    /**
     * @brief スコア計算クラス
     *
     */
    class Score {
    private:
        //! 重み
        double weight_;
        //! 評価関数
        std::function<double(const local_path_data_t&)> func_;
        /**
         * @brief 正規化
         *
         * @param data 評価値
         * @return std::vector<double> 正規化評価値
         */
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
        /**
         * @brief Construct a new Score object
         *
         * @param weight 重み
         * @param func 評価関数
         */
        Score(double weight, std::function<double(const local_path_data_t&)> func) : weight_(weight), func_(func) {}
        /**
         * @brief スコア計算
         *
         * @param path 候補パスベクトル
         * @return Eigen::VectorXd 評価値
         */
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

    //! 現在速度
    Eigen::Vector3d current_vel_;
    //! 現在位置
    Eigen::Vector3d current_pos_;
    //! 共分散ノルム
    double covariance_norm_ = 0;
    //! UMapタイムアウト時刻
    double umap_timeout_time_ = 0;

    //! グローバルパス
    std::vector<geometry_msgs::msg::PoseStamped> global_path_;
    //! ローカル目標位置
    Eigen::Vector3d local_target_pos_;
    //! 地図障害物
    std::optional<kd_tree::EigenVectorTree<2>> map_grid_;

    //! 近傍インデックス
    int near_index_ = 0;
    //! 初期角度制御フラグ
    bool is_start_angle_control_ = true;

    //! 制御周期
    double control_period_;
    //! 共分散ゲイン
    double covariance_gain_;
    //! 最大速度
    double max_vel_;
    //! 最小機体速度
    double minimum_max_vel_;
    //! 最大加速度
    double max_acc_;
    //! 最大角速度
    double max_angle_vel_;
    //! 最小機体角速度
    double minimum_max_angle_vel_;
    //! 最大角加速度
    double max_angle_acc_;
    //! カーブ検索インデックス数
    int serch_curve_index_num_;
    //! カーブ速度レート
    double curve_vel_rate_;
    //! 速度分割数
    int vel_split_num_;
    //! シミュレーション時間
    double simulation_time_;
    //! シミュレーションステップ数
    int simulation_step_;
    //! 衝突地図障害物距離
    double collision_map_distance_;
    //! 無視地図障害物距離
    double ignore_map_distance_;
    //! UMapタイムアウト停止時間閾値
    double umap_timeout_stop_time_;

    //! ローカル目標位置スコアの重み
    double score_local_target_distance_waight_;
    //! ローカル目標角度スコアの重み
    double score_local_target_angle_waight_;
    //! 地図障害物距離スコアの重み
    double score_map_distance_waight_;
    //! 積分誤差スコアの重み
    double score_integral_error_waight_;

    //! 目標位置近傍制御への切り替え距離
    double near_goal_control_distance_;
    //! 目標位置近傍制御位置ゲイン
    double near_goal_control_pos_kp_;
    //! 目標位置近傍制御角度ゲイン
    double near_goal_control_angle_kp_;

    //! 目標位置収束判定距離
    double goal_distance_;

    //! 現在取れる最大速度
    double current_max_vel_;
    //! 現在取れる最大角速度
    double current_max_angle_vel_;

    //! スコアクラスベクトル
    std::vector<Score> scores_;

public:
    /**
     * @brief Construct a new Route Following object
     *
     * @param options
     */
    RouteFollowing(const rclcpp::NodeOptions& options) : RouteFollowing("", options) {}
    /**
     * @brief Construct a new Route Following object
     *
     * @param name_space
     * @param options
     */
    RouteFollowing(const std::string& name_space = "", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("route_following_node", name_space, options)
    {
        using namespace std::chrono_literals;

        declare_parameter<double>("control_period", 0.1);
        control_period_ = get_parameter("control_period").as_double();
        declare_parameter<double>("covariance_gain", 1.0);
        covariance_gain_ = get_parameter("covariance_gain").as_double();
        declare_parameter<double>("max_vel", 1.0);
        max_vel_ = get_parameter("max_vel").as_double();
        declare_parameter<double>("minimum_max_vel", 0.1);
        minimum_max_vel_ = get_parameter("minimum_max_vel").as_double();
        declare_parameter<double>("max_acc", 1.0);
        max_acc_ = get_parameter("max_acc").as_double();
        declare_parameter<double>("max_angle_vel", 1.0);
        max_angle_vel_ = get_parameter("max_angle_vel").as_double();
        declare_parameter<double>("minimum_max_angle_vel", 0.1);
        minimum_max_angle_vel_ = get_parameter("minimum_max_angle_vel").as_double();
        declare_parameter<double>("max_angle_acc", 1.0);
        max_angle_acc_ = get_parameter("max_angle_acc").as_double();
        declare_parameter<int>("serch_curve_index_num", 1);
        serch_curve_index_num_ = get_parameter("serch_curve_index_num").as_int();
        declare_parameter<double>("curve_vel_rate", 1.0);
        curve_vel_rate_ = get_parameter("curve_vel_rate").as_double();
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
        declare_parameter<double>("umap_timeout_stop_time", 1.0);
        umap_timeout_stop_time_ = get_parameter("umap_timeout_stop_time").as_double();

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

        // 評価関数設定
        scores_.push_back(Score(score_local_target_distance_waight_, [&](const local_path_data_t& path) { return calc_score_local_target_distance(path); }));
        scores_.push_back(Score(score_local_target_angle_waight_, [&](const local_path_data_t& path) { return calc_score_local_target_angle(path); }));
        scores_.push_back(Score(score_map_distance_waight_, [&](const local_path_data_t& path) { return calc_score_map_distance(path); }));
        scores_.push_back(Score(score_integral_error_waight_, [&](const local_path_data_t& path) { return calc_score_integral_error(path); }));

        static auto velocity_pub = create_publisher<geometry_msgs::msg::Twist>("target_vel", 5);
        static auto candidate_path_pub = create_publisher<nav_msgs::msg::Path>("local_path_candidate", rclcpp::QoS(10).reliable());
        static auto local_path_pub = create_publisher<nav_msgs::msg::Path>("local_path", rclcpp::QoS(10).reliable());
        static auto local_target_pos_pub = create_publisher<geometry_msgs::msg::PoseStamped>("local_target_pos", rclcpp::QoS(10).reliable());
        static auto route_follow_state_pub = create_publisher<std_msgs::msg::String>("route_follow_state", rclcpp::QoS(10).reliable());

        static auto localization_sub = create_subscription<nav_msgs::msg::Odometry>("ekf_odom", rclcpp::QoS(10).reliable(), [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_pos_ = make_eigen_vector3d(msg->pose.pose);
            current_vel_ = make_eigen_vector3d(msg->twist.twist);
            current_vel_.head<2>() = rotate_2d(current_vel_.head<2>(), current_pos_.z());
            covariance_norm_ = std::hypot(msg->pose.covariance[XYZRPY_COV_IDX::X_X], msg->pose.covariance[XYZRPY_COV_IDX::Y_Y], msg->pose.covariance[XYZRPY_COV_IDX::YAW_YAW]);
        });
        static auto global_path_sub = create_subscription<nav_msgs::msg::Path>("global_path", rclcpp::QoS(10).reliable(), [&](const nav_msgs::msg::Path::SharedPtr msg) {
            global_path_ = msg->poses;
            near_index_ = 0;
            is_start_angle_control_ = true;
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

        static auto umap_timeout_sub = create_subscription<std_msgs::msg::Header>("umap_timeout", rclcpp::QoS(10).reliable(), [&](const std_msgs::msg::Header::SharedPtr msg) { umap_timeout_time_ = rclcpp::Time(msg->stamp).seconds(); });

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
            std_msgs::msg::String state_msg;
            // グローバルパスがない場合
            if (global_path_.empty()) {
                geometry_msgs::msg::Twist vel_msg;
                vel_msg.linear.x = 0;
                vel_msg.linear.y = 0;
                vel_msg.angular.z = 0;
                velocity_pub->publish(vel_msg);
                state_msg.data = "wait_global_path";
                route_follow_state_pub->publish(state_msg);
                return;
            }

            // 初期角度制御モード
            if (is_start_angle_control_) {
                Eigen::Vector3d e = make_eigen_vector3d(global_path_.front().pose) - current_pos_;
                e.z() = normalize_angle(e.z());
                auto target_vel = Eigen::Vector3d(0, 0, e.z() * near_goal_control_angle_kp_);
                target_vel.z() = std::clamp(target_vel.z(), -max_angle_vel_, max_angle_vel_);
                geometry_msgs::msg::Twist vel_msg;
                vel_msg = make_twist(target_vel);
                velocity_pub->publish(vel_msg);
                state_msg.data = "start_angle_control";
                route_follow_state_pub->publish(state_msg);
                if (std::abs(e.z()) < 0.1) {
                    is_start_angle_control_ = false;
                }
                return;
            }

            // ゴールに到達した場合
            if (auto goal_pos = make_eigen_vector3d(global_path_.back().pose); (goal_pos - current_pos_).head<2>().norm() < goal_distance_) {
                global_path_.clear();
                geometry_msgs::msg::Twist vel_msg;
                vel_msg.linear.x = 0;
                vel_msg.linear.y = 0;
                vel_msg.angular.z = 0;
                velocity_pub->publish(vel_msg);
                state_msg.data = "goal";
                route_follow_state_pub->publish(state_msg);
                return;
            }
            // ゴールに近づいた場合
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
                state_msg.data = "near_goal_control";
                route_follow_state_pub->publish(state_msg);
                return;
            }

            auto start_time = this->get_clock()->now();

            {
                // グローバルパス上の最近傍インデックスを探索
                double min_e = std::numeric_limits<double>::infinity();
                for (size_t i = near_index_; i < global_path_.size(); i++) {
                    auto pos = Eigen::Vector2d(global_path_[i].pose.position.x, global_path_[i].pose.position.y);
                    double e = (pos - current_pos_.head<2>()).norm();
                    if (e < min_e) {
                        min_e = e;
                        near_index_ = i;
                    }
                }

                current_max_vel_ = max_vel_ / (1 + covariance_gain_ * covariance_norm_);
                geometry_msgs::msg::PoseStamped before_pose = global_path_[std::max(near_index_ - serch_curve_index_num_, 0)];

                // カーブでは減速する
                double rate = 1;
                for (int i = -serch_curve_index_num_; i <= serch_curve_index_num_; i++) {
                    auto pose = global_path_[std::clamp(near_index_ + i, 0, static_cast<int>(global_path_.size()) - 1)];
                    if (pose.pose.position.x != before_pose.pose.position.x && pose.pose.position.y != before_pose.pose.position.y) {
                        rate = std::min(rate, (1.0 - curve_vel_rate_) / serch_curve_index_num_ * std::abs(i) + curve_vel_rate_);
                    }
                    before_pose = pose;
                }
                current_max_vel_ = std::max(current_max_vel_ * rate, minimum_max_vel_);
                current_max_angle_vel_ = std::max(max_angle_vel_ / (1 + covariance_gain_ * covariance_norm_), minimum_max_angle_vel_);

                // UMapタイムアウト時は停止
                if ((this->get_clock()->now().seconds() - umap_timeout_time_) < umap_timeout_stop_time_) {
                    current_max_vel_ = 0;
                    current_max_vel_ = 0;
                }

                // ローカル目標位置を設定
                int target_index = near_index_;
                double route_distance = 0;
                auto prev_pos = Eigen::Vector2d(global_path_[near_index_].pose.position.x, global_path_[near_index_].pose.position.y);
                for (size_t i = target_index; i < global_path_.size(); i++) {
                    auto pos = Eigen::Vector2d(global_path_[i].pose.position.x, global_path_[i].pose.position.y);
                    route_distance += (pos - prev_pos).norm();
                    prev_pos = pos;
                    target_index = i;
                    if (route_distance >= current_max_vel_ * simulation_time_) {
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

            auto choices_path = make_choices_path();
            for (auto& p : choices_path) {
                path_publish(p.path, candidate_path_pub);
            }

            // 経路ごとのスコア計算
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
            state_msg.data = "dwa_control";
            route_follow_state_pub->publish(state_msg);
        });
    }

private:
    /**
     * @brief 移動位置シミュレーション
     *
     * @param vel 指示速度
     * @param pos 現在位置
     * @return std::vector<Eigen::Vector3d> 移動経路
     */
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

    /**
     * @brief 候補パス作成
     *
     * @return std::vector<local_path_data_t> 候補パス
     */
    std::vector<local_path_data_t> make_choices_path()
    {
        auto rotate_vel = rotate_2d(current_vel_.head<2>(), -current_pos_.z());
        std::vector<local_path_data_t> choices_path;
        int half_vel_split_num = vel_split_num_ / 2;
        for (int i = -half_vel_split_num; i < half_vel_split_num; i++) {
            double x = rotate_vel.x() + max_acc_ * control_period_ / half_vel_split_num * i;
            for (int j = -half_vel_split_num; j < half_vel_split_num; j++) {
                double y = rotate_vel.y() + max_acc_ * control_period_ / half_vel_split_num * j;
                for (int k = -half_vel_split_num; k < half_vel_split_num; k++) {
                    double z = current_vel_.z() + max_angle_acc_ * control_period_ / half_vel_split_num * k;
                    if ((std::hypot(x, y) <= current_max_vel_ && std::abs(z) <= current_max_angle_vel_) || (std::hypot(x, y) < rotate_vel.norm() && std::abs(z) < std::abs(current_vel_.z()))) {
                        choices_path.push_back(local_path_data_t(Eigen::Vector3d(x, y, z), simulation_pos(Eigen::Vector3d(x, y, z), current_pos_)));
                    }
                }
            }
        }
        if (choices_path.empty()) {
            choices_path.push_back(local_path_data_t(Eigen::Vector3d(0, 0, 0), simulation_pos(Eigen::Vector3d(0, 0, 0), current_pos_)));
        }
        return choices_path;
    }

    /**
     * @brief ローカル目標位置評価関数
     *
     * @param path 経路
     * @return double スコア
     */
    double calc_score_local_target_distance(const local_path_data_t& path) const { return -(local_target_pos_ - path.path.back()).head<2>().norm(); }

    /**
     * @brief ローカル目標角度評価関数
     *
     * @param path 経路
     * @return double スコア
     */
    double calc_score_local_target_angle(const local_path_data_t& path) const { return -std::abs(normalize_angle((local_target_pos_ - path.path.back()).z())); }

    /**
     * @brief 地図障害物距離評価関数
     *
     * @param path 経路
     * @return double スコア
     */
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

    /**
     * @brief 積分誤差評価関数
     *
     * @param path 経路
     * @return double スコア
     */
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