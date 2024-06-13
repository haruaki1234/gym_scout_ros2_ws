/**
 * @file hyper_parameters.hpp
 * @author Takuma Nakao
 * @brief EKF用ハイパーパラメータ
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <string>

namespace tlab
{

/**
 * @brief EKF用ハイパーパラメータクラス
 *
 */
class HyperParameters {
public:
    /**
     * @brief Construct a new Hyper Parameters object
     *
     * @param node ROS2ノードポインタ
     */
    explicit HyperParameters(rclcpp::Node* node) :
        show_debug_info(node->declare_parameter("show_debug_info", false)),             //
        ekf_rate(node->declare_parameter("predict_frequency", 50.0)),                   //
        ekf_dt(1.0 / std::max(ekf_rate, 0.1)),                                          //
        tf_rate_(node->declare_parameter("tf_rate", 10.0)),                             //
        publish_tf_(node->declare_parameter("publish_tf", true)),                       //
        extend_state_step(node->declare_parameter("extend_state_step", 50)),            //
        pose_frame_id(node->declare_parameter("pose_frame_id", std::string("map"))),    //
        pose_additional_delay(node->declare_parameter("pose_additional_delay", 0.0)),   //
        pose_gate_dist(node->declare_parameter("pose_gate_dist", 10000.0)),             //
        pose_smoothing_steps(node->declare_parameter("pose_smoothing_steps", 5)),       //
        twist_additional_delay(node->declare_parameter("twist_additional_delay", 0.0)), //
        twist_gate_dist(node->declare_parameter("twist_gate_dist", 10000.0)),           //
        twist_smoothing_steps(node->declare_parameter("twist_smoothing_steps", 2)),     //
        proc_stddev_vx_c(node->declare_parameter("proc_stddev_vx_c", 5.0)),             //
        proc_stddev_vy_c(node->declare_parameter("proc_stddev_vy_c", 5.0)),             //
        proc_stddev_wz_c(node->declare_parameter("proc_stddev_wz_c", 1.0)),             //
        proc_stddev_yaw_c(node->declare_parameter("proc_stddev_yaw_c", 0.005)),         //
        twist_stddev_vx_c(node->declare_parameter("twist_stddev_vx_c", 0.1)),           //
        twist_stddev_vy_c(node->declare_parameter("twist_stddev_vy_c", 0.1)),           //
        twist_stddev_wz_c(node->declare_parameter("twist_stddev_wz_c", 0.1))            //
    {
    }

    //! デバッグ情報を表示するかどうか
    const bool show_debug_info;
    //! EKFの予測周波数
    const double ekf_rate;
    //! EKFの予測周期
    const double ekf_dt;
    //! TFの更新周波数
    const double tf_rate_;
    //! パブリッシュの周波数
    const bool publish_tf_;
    //! 最大遅れ保証ステップ数
    const int extend_state_step;
    //! ポーズフレームID
    const std::string pose_frame_id;
    //! 位置姿勢計測の追加遅延
    const double pose_additional_delay;
    //! 位置姿勢のマハラノビス距離制限
    const double pose_gate_dist;
    //! 位置姿勢のスムージングステップ数
    const int pose_smoothing_steps;
    //! 速度計測の追加遅延
    const double twist_additional_delay;
    //! 速度のマハラノビス距離制限
    const double twist_gate_dist;
    //! 速度のスムージングステップ数
    const int twist_smoothing_steps;
    //! vx process noise
    const double proc_stddev_vx_c;
    //! vy process noise
    const double proc_stddev_vy_c;
    //! wz process noise
    const double proc_stddev_wz_c;
    //! yaw process noise
    const double proc_stddev_yaw_c;
    //! twist vx noise
    const double twist_stddev_vx_c;
    //! twist vy noise
    const double twist_stddev_vy_c;
    //! twist wz noise
    const double twist_stddev_wz_c;
};

} // namespace tlab