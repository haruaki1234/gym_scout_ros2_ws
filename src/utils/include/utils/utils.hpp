/**
 * @file utils.hpp
 * @author Takuma Nakao
 * @brief ROS2ユーティリティ関数
 * @date 2024-05-23
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>

#include "./transform.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/transform.hpp"

namespace tlab
{

/**
 * @brief Poseメッセージを作成
 *
 * @param x 位置X
 * @param y 位置Y
 * @param theta 角度YAW
 * @return geometry_msgs::msg::Pose メッセージ
 */
static geometry_msgs::msg::Pose make_pose(double x, double y, double theta)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation.z = std::sin(theta / 2);
    pose.orientation.w = std::cos(theta / 2);
    return pose;
}

/**
 * @brief Poseメッセージを作成
 *
 * @param pos tlab::Transformによる位置姿勢
 * @return geometry_msgs::msg::Pose メッセージ
 */
static geometry_msgs::msg::Pose make_pose(const tlab::Transform& pos) { return make_pose(pos.x, pos.y, pos.theta); }

/**
 * @brief Poseメッセージを作成
 *
 * @param pos Eigen::Vector3dによる位置姿勢
 * @return geometry_msgs::msg::Pose メッセージ
 */
static geometry_msgs::msg::Pose make_pose(const Eigen::Vector3d& pos) { return make_pose(pos.x(), pos.y(), pos.z()); }

/**
 * @brief Transformメッセージを作成
 *
 * @param x 位置X
 * @param y 位置Y
 * @param theta 角度YAW
 * @return geometry_msgs::msg::Transform メッセージ
 */
static geometry_msgs::msg::Transform make_geometry_transform(double x, double y, double theta)
{
    geometry_msgs::msg::Transform transform;
    transform.translation.x = x;
    transform.translation.y = y;
    transform.rotation.z = std::sin(theta / 2);
    transform.rotation.w = std::cos(theta / 2);
    return transform;
}

/**
 * @brief Transformメッセージを作成
 *
 * @param pos tlab::Transformによる位置姿勢
 * @return geometry_msgs::msg::Transform メッセージ
 */
static geometry_msgs::msg::Transform make_geometry_transform(const tlab::Transform& pos) { return make_geometry_transform(pos.x, pos.y, pos.theta); }

/**
 * @brief Transformメッセージを作成
 *
 * @param pos Eigen::Vector3dによる位置姿勢
 * @return geometry_msgs::msg::Transform メッセージ
 */
static geometry_msgs::msg::Transform make_geometry_transform(const Eigen::Vector3d& pos) { return make_geometry_transform(pos.x(), pos.y(), pos.z()); }

/**
 * @brief Twistメッセージを作成
 *
 * @param x 速度X
 * @param y 速度Y
 * @param theta 角速度YAW
 * @return geometry_msgs::msg::Twist メッセージ
 */
static geometry_msgs::msg::Twist make_twist(double x, double y, double theta)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x = x;
    twist.linear.y = y;
    twist.angular.z = theta;
    return twist;
}

/**
 * @brief Twistメッセージを作成
 *
 * @param vel tlab::Transformによる速度
 * @return geometry_msgs::msg::Twist メッセージ
 */
static geometry_msgs::msg::Twist make_twist(const tlab::Transform& vel) { return make_twist(vel.x, vel.y, vel.theta); }

/**
 * @brief Twistメッセージを作成
 *
 * @param vel Eigen::Vector3dによる速度
 * @return geometry_msgs::msg::Twist メッセージ
 */
static geometry_msgs::msg::Twist make_twist(const Eigen::Vector3d& vel) { return make_twist(vel.x(), vel.y(), vel.z()); }

/**
 * @brief Accelメッセージを作成
 *
 * @param x 加速度X
 * @param y 加速度Y
 * @param theta 角加速度YAW
 * @return geometry_msgs::msg::Accel メッセージ
 */
static geometry_msgs::msg::Accel make_accel(double x, double y, double theta)
{
    geometry_msgs::msg::Accel accel;
    accel.linear.x = x;
    accel.linear.y = y;
    accel.angular.z = theta;
    return accel;
}

/**
 * @brief Accelメッセージを作成
 *
 * @param acc tlab::Transformによる加速度
 * @return geometry_msgs::msg::Accel メッセージ
 */
static geometry_msgs::msg::Accel make_accel(const tlab::Transform& acc) { return make_accel(acc.x, acc.y, acc.theta); }

/**
 * @brief Accelメッセージを作成
 *
 * @param acc Eigen::Vector3dによる加速度
 * @return geometry_msgs::msg::Accel メッセージ
 */
static geometry_msgs::msg::Accel make_accel(const Eigen::Vector3d& acc) { return make_accel(acc.x(), acc.y(), acc.z()); }

/**
 * @brief Poseメッセージからtlab::Transformを作成
 *
 * @param pose メッセージ
 * @return tlab::Transform 位置姿勢
 */
static tlab::Transform make_tlab_transform(const geometry_msgs::msg::Pose& pose) { return tlab::Transform(pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)); }

/**
 * @brief PoseメッセージからEigen::Vector3dを作成
 *
 * @param pose メッセージ
 * @return Eigen::Vector3d 位置姿勢
 */
static Eigen::Vector3d make_eigen_vector3d(const geometry_msgs::msg::Pose& pose) { return Eigen::Vector3d(pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)); }

/**
 * @brief Transformメッセージからtlab::Transformを作成
 *
 * @param transform メッセージ
 * @return tlab::Transform 位置姿勢
 */
static tlab::Transform make_tlab_transform(const geometry_msgs::msg::Transform& transform) { return tlab::Transform(transform.translation.x, transform.translation.y, tf2::getYaw(transform.rotation)); }

/**
 * @brief TransformメッセージからEigen::Vector3dを作成
 *
 * @param transform メッセージ
 * @return Eigen::Vector3d 位置姿勢
 */
static Eigen::Vector3d make_eigen_vector3d(const geometry_msgs::msg::Transform& transform) { return Eigen::Vector3d(transform.translation.x, transform.translation.y, tf2::getYaw(transform.rotation)); }

/**
 * @brief Twistメッセージからtlab::Transformを作成
 *
 * @param twist メッセージ
 * @return tlab::Transform 速度
 */
static tlab::Transform make_tlab_transform(const geometry_msgs::msg::Twist& twist) { return tlab::Transform(twist.linear.x, twist.linear.y, twist.angular.z); }

/**
 * @brief TwistメッセージからEigen::Vector3dを作成
 *
 * @param twist メッセージ
 * @return Eigen::Vector3d 速度
 */
static Eigen::Vector3d make_eigen_vector3d(const geometry_msgs::msg::Twist& twist) { return Eigen::Vector3d(twist.linear.x, twist.linear.y, twist.angular.z); }

/**
 * @brief Accelメッセージからtlab::Transformを作成
 *
 * @param accel メッセージ
 * @return tlab::Transform 加速度
 */
static tlab::Transform make_tlab_transform(const geometry_msgs::msg::Accel& accel) { return tlab::Transform(accel.linear.x, accel.linear.y, accel.angular.z); }

/**
 * @brief AccelメッセージからEigen::Vector3dを作成
 *
 * @param accel メッセージ
 * @return Eigen::Vector3d 加速度
 */
static Eigen::Vector3d make_eigen_vector3d(const geometry_msgs::msg::Accel& accel) { return Eigen::Vector3d(accel.linear.x, accel.linear.y, accel.angular.z); }

/**
 * @brief Eigen::Vector2dを回転
 *
 * @param vec 回転させるベクトル
 * @param theta 回転角度[rad]
 * @return Eigen::Vector2d 回転後のベクトル
 */
static Eigen::Vector2d rotate_2d(const Eigen::Vector2d& vec, double theta)
{
    Eigen::Matrix2d rot;
    rot << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
    return rot * vec;
}

} // namespace tlab