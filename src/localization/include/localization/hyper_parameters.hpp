#pragma once

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <string>

namespace tlab
{

class HyperParameters {
public:
    explicit HyperParameters(rclcpp::Node* node) :
        show_debug_info(node->declare_parameter("show_debug_info", false)),                      //
        ekf_rate(node->declare_parameter("predict_frequency", 50.0)),                            //
        ekf_dt(1.0 / std::max(ekf_rate, 0.1)),                                                   //
        tf_rate_(node->declare_parameter("tf_rate", 10.0)),                                      //
        publish_tf_(node->declare_parameter("publish_tf", true)),                                //
        enable_yaw_bias_estimation(node->declare_parameter("enable_yaw_bias_estimation", true)), //
        extend_state_step(node->declare_parameter("extend_state_step", 50)),                     //
        pose_frame_id(node->declare_parameter("pose_frame_id", std::string("map"))),             //
        pose_additional_delay(node->declare_parameter("pose_additional_delay", 0.0)),            //
        pose_gate_dist(node->declare_parameter("pose_gate_dist", 10000.0)),                      //
        pose_smoothing_steps(node->declare_parameter("pose_smoothing_steps", 5)),                //
        twist_additional_delay(node->declare_parameter("twist_additional_delay", 0.0)),          //
        twist_gate_dist(node->declare_parameter("twist_gate_dist", 10000.0)),                    //
        twist_smoothing_steps(node->declare_parameter("twist_smoothing_steps", 2)),              //
        proc_stddev_vx_c(node->declare_parameter("proc_stddev_vx_c", 5.0)),                      //
        proc_stddev_vy_c(node->declare_parameter("proc_stddev_vy_c", 5.0)),                      //
        proc_stddev_wz_c(node->declare_parameter("proc_stddev_wz_c", 1.0)),                      //
        proc_stddev_yaw_c(node->declare_parameter("proc_stddev_yaw_c", 0.005)),                  //
        twist_stddev_vx_c(node->declare_parameter("twist_stddev_vx_c", 0.1)),                    //
        twist_stddev_vy_c(node->declare_parameter("twist_stddev_vy_c", 0.1)),                    //
        twist_stddev_wz_c(node->declare_parameter("twist_stddev_wz_c", 0.1))                     //
    {
    }

    const bool show_debug_info;
    const double ekf_rate;
    const double ekf_dt;
    const double tf_rate_;
    const bool publish_tf_;
    const bool enable_yaw_bias_estimation;
    const int extend_state_step;
    const std::string pose_frame_id;
    const double pose_additional_delay;
    const double pose_gate_dist;
    const int pose_smoothing_steps;
    const double twist_additional_delay;
    const double twist_gate_dist;
    const int twist_smoothing_steps;
    const double proc_stddev_vx_c;  //!< @brief  vx process noise
    const double proc_stddev_vy_c;  //!< @brief  vy process noise
    const double proc_stddev_wz_c;  //!< @brief  wz process noise
    const double proc_stddev_yaw_c; //!< @brief  yaw process noise
    const double twist_stddev_vx_c;
    const double twist_stddev_vy_c;
    const double twist_stddev_wz_c;
};

} // namespace tlab