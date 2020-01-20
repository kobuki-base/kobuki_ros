/**
 * @file /kobuki_node/src/node/odometry.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <memory>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <ecl/geometry.hpp>
#include <ecl/linear_algebra.hpp>

#include "kobuki_node/odometry.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki_node
{

/*****************************************************************************
** Implementation
*****************************************************************************/

Odometry::Odometry(double cmd_vel_timeout, const std::string & odom_frame, const std::string & base_frame, bool publish_tf, bool use_imu_heading) :
  cmd_vel_timeout_(cmd_vel_timeout),
  odom_frame_(odom_frame),
  base_frame_(base_frame),
  publish_tf_(publish_tf),
  use_imu_heading_(use_imu_heading)
{
  pose_.setIdentity();
}

bool Odometry::commandTimeout(const rclcpp::Time & now) const {
  if ((last_cmd_time_.nanoseconds() != 0) && ((now - last_cmd_time_) > cmd_vel_timeout_)) {
    return true;
  }
  else {
    return false;
  }
}

geometry_msgs::msg::Quaternion Odometry::update(const ecl::LegacyPose2D<double> &pose_update, ecl::linear_algebra::Vector3d &pose_update_rates,
                                                double imu_heading, double imu_angular_velocity) {
  pose_ *= pose_update;

  if (use_imu_heading_) {
    // Overwite with gyro heading data
    pose_.heading(imu_heading);
    pose_update_rates[2] = imu_angular_velocity;
  }

  //since all ros tf odometry is 6DOF we'll need a quaternion created from yaw
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose_.heading());
  geometry_msgs::msg::Quaternion odom_quat;
  odom_quat.x = q.x();
  odom_quat.y = q.y();
  odom_quat.z = q.z();
  odom_quat.w = q.w();

  return odom_quat;
}

/*****************************************************************************
** Private Implementation
*****************************************************************************/

std::unique_ptr<geometry_msgs::msg::TransformStamped> Odometry::getTransform(const geometry_msgs::msg::Quaternion &odom_quat, const rclcpp::Time & now)
{
  if (publish_tf_ == false) {
    return nullptr;
  }

  auto odom_trans = std::make_unique<geometry_msgs::msg::TransformStamped>();

  odom_trans->header.frame_id = odom_frame_;
  odom_trans->child_frame_id = base_frame_;
  odom_trans->header.stamp = now;
  odom_trans->transform.translation.x = pose_.x();
  odom_trans->transform.translation.y = pose_.y();
  odom_trans->transform.translation.z = 0.0;
  odom_trans->transform.rotation = odom_quat;

  return std::move(odom_trans);
}

std::unique_ptr<nav_msgs::msg::Odometry> Odometry::getOdometry(const geometry_msgs::msg::Quaternion &odom_quat,
                                                               const ecl::linear_algebra::Vector3d &pose_update_rates,
                                                               const rclcpp::Time & now)
{
  // Publish as unique pointer to leverage the zero-copy pub/sub feature
  auto odom = std::make_unique<nav_msgs::msg::Odometry>();

  // Header
  odom->header.stamp = now;
  odom->header.frame_id = odom_frame_;
  odom->child_frame_id = base_frame_;

  // Position
  odom->pose.pose.position.x = pose_.x();
  odom->pose.pose.position.y = pose_.y();
  odom->pose.pose.position.z = 0.0;
  odom->pose.pose.orientation = odom_quat;

  // Velocity
  odom->twist.twist.linear.x = pose_update_rates[0];
  odom->twist.twist.linear.y = pose_update_rates[1];
  odom->twist.twist.angular.z = pose_update_rates[2];

  // Pose covariance (required by robot_pose_ekf) TODO: publish realistic values
  // Odometry yaw covariance must be much bigger than the covariance provided
  // by the imu, as the later takes much better measures
  odom->pose.covariance[0]  = 0.1;
  odom->pose.covariance[7]  = 0.1;
  odom->pose.covariance[35] = use_imu_heading_ ? 0.05 : 0.2;

  odom->pose.covariance[14] = 1e10; // set a non-zero covariance on unused
  odom->pose.covariance[21] = 1e10; // dimensions (z, pitch and roll); this
  odom->pose.covariance[28] = 1e10; // is a requirement of robot_pose_ekf

  return std::move(odom);
}

} // namespace kobuki
