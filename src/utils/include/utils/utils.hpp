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

static geometry_msgs::msg::Pose make_pose(double x, double y, double theta)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation.z = std::sin(theta / 2);
    pose.orientation.w = std::cos(theta / 2);
    return pose;
}

static geometry_msgs::msg::Pose make_pose(const tlab::Transform& pos) { return make_pose(pos.x, pos.y, pos.theta); }
static geometry_msgs::msg::Pose make_pose(const Eigen::Vector3d& pos) { return make_pose(pos.x(), pos.y(), pos.z()); }

static geometry_msgs::msg::Transform make_geometry_transform(double x, double y, double theta)
{
    geometry_msgs::msg::Transform transform;
    transform.translation.x = x;
    transform.translation.y = y;
    transform.rotation.z = std::sin(theta / 2);
    transform.rotation.w = std::cos(theta / 2);
    return transform;
}

static geometry_msgs::msg::Transform make_geometry_transform(const tlab::Transform& pos) { return make_geometry_transform(pos.x, pos.y, pos.theta); }
static geometry_msgs::msg::Transform make_geometry_transform(const Eigen::Vector3d& pos) { return make_geometry_transform(pos.x(), pos.y(), pos.z()); }

static geometry_msgs::msg::Twist make_twist(double x, double y, double theta)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x = x;
    twist.linear.y = y;
    twist.angular.z = theta;
    return twist;
}

static geometry_msgs::msg::Twist make_twist(const tlab::Transform& vel) { return make_twist(vel.x, vel.y, vel.theta); }
static geometry_msgs::msg::Twist make_twist(const Eigen::Vector3d& vel) { return make_twist(vel.x(), vel.y(), vel.z()); }

static geometry_msgs::msg::Accel make_accel(double x, double y, double theta)
{
    geometry_msgs::msg::Accel accel;
    accel.linear.x = x;
    accel.linear.y = y;
    accel.angular.z = theta;
    return accel;
}

static geometry_msgs::msg::Accel make_accel(const tlab::Transform& acc) { return make_accel(acc.x, acc.y, acc.theta); }
static geometry_msgs::msg::Accel make_accel(const Eigen::Vector3d& acc) { return make_accel(acc.x(), acc.y(), acc.z()); }

static tlab::Transform make_tlab_transform(const geometry_msgs::msg::Pose& pose) { return tlab::Transform(pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)); }
static Eigen::Vector3d make_eigen_vector3d(const geometry_msgs::msg::Pose& pose) { return Eigen::Vector3d(pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)); }

static tlab::Transform make_tlab_transform(const geometry_msgs::msg::Transform& transform) { return tlab::Transform(transform.translation.x, transform.translation.y, tf2::getYaw(transform.rotation)); }
static Eigen::Vector3d make_eigen_vector3d(const geometry_msgs::msg::Transform& transform) { return Eigen::Vector3d(transform.translation.x, transform.translation.y, tf2::getYaw(transform.rotation)); }

static tlab::Transform make_tlab_transform(const geometry_msgs::msg::Twist& twist) { return tlab::Transform(twist.linear.x, twist.linear.y, twist.angular.z); }
static Eigen::Vector3d make_eigen_vector3d(const geometry_msgs::msg::Twist& twist) { return Eigen::Vector3d(twist.linear.x, twist.linear.y, twist.angular.z); }

static tlab::Transform make_tlab_transform(const geometry_msgs::msg::Accel& accel) { return tlab::Transform(accel.linear.x, accel.linear.y, accel.angular.z); }
static Eigen::Vector3d make_eigen_vector3d(const geometry_msgs::msg::Accel& accel) { return Eigen::Vector3d(accel.linear.x, accel.linear.y, accel.angular.z); }

static Eigen::Vector2d rotate_2d(const Eigen::Vector2d& vec, double theta)
{
    Eigen::Matrix2d rot;
    rot << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
    return rot * vec;
}

} // namespace tlab