#pragma once

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

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
    pose.orientation.z = sin(theta / 2);
    pose.orientation.w = cos(theta / 2);
    return pose;
}

static geometry_msgs::msg::Pose make_pose(tlab::Transform pos) { return make_pose(pos.x, pos.y, pos.theta); }

static geometry_msgs::msg::Transform make_geometry_transform(double x, double y, double theta)
{
    geometry_msgs::msg::Transform transform;
    transform.translation.x = x;
    transform.translation.y = y;
    transform.rotation.z = sin(theta / 2);
    transform.rotation.w = cos(theta / 2);
    return transform;
}

static geometry_msgs::msg::Transform make_geometry_transform(tlab::Transform pos) { return make_geometry_transform(pos.x, pos.y, pos.theta); }

static geometry_msgs::msg::Twist make_twist(double x, double y, double theta)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x = x;
    twist.linear.y = y;
    twist.angular.z = theta;
    return twist;
}

static geometry_msgs::msg::Twist make_twist(tlab::Transform vel) { return make_twist(vel.x, vel.y, vel.theta); }

static geometry_msgs::msg::Accel make_accel(double x, double y, double theta)
{
    geometry_msgs::msg::Accel accel;
    accel.linear.x = x;
    accel.linear.y = y;
    accel.angular.z = theta;
    return accel;
}

static geometry_msgs::msg::Accel make_accel(tlab::Transform acc) { return make_accel(acc.x, acc.y, acc.theta); }

static tlab::Transform make_tlab_transform(geometry_msgs::msg::Pose pose) { return tlab::Transform(pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)); }

static tlab::Transform make_tlab_transform(geometry_msgs::msg::Transform transform) { return tlab::Transform(transform.translation.x, transform.translation.y, tf2::getYaw(transform.rotation)); }

static tlab::Transform make_tlab_transform(geometry_msgs::msg::Twist twist) { return tlab::Transform(twist.linear.x, twist.linear.y, twist.angular.z); }

static tlab::Transform make_tlab_transform(geometry_msgs::msg::Accel accel) { return tlab::Transform(accel.linear.x, accel.linear.y, accel.angular.z); }

} // namespace tlab