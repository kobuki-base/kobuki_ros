/**
 * @file /kobuki_node/include/kobuki_node/odometry.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef KOBUKI_NODE_ODOMETRY_HPP_
#define KOBUKI_NODE_ODOMETRY_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ecl/geometry/legacy_pose2d.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kobuki_node
{

/*****************************************************************************
** Interfaces
*****************************************************************************/

/**
 * @brief  Odometry for the kobuki node.
 **/
class Odometry final {
public:
  explicit Odometry(double cmd_vel_timeout, const std::string & odom_frame, const std::string & base_frame, bool publish_tf, bool use_imu_heading);
  bool commandTimeout(const rclcpp::Time & now) const;
  geometry_msgs::msg::Quaternion update(const ecl::LegacyPose2D<double> &pose_update, ecl::linear_algebra::Vector3d &pose_update_rates,
                                        double imu_heading, double imu_angular_velocity);
  void resetOdometry() { pose_.setIdentity(); }
  const rclcpp::Duration& timeout() const { return cmd_vel_timeout_; }
  void resetTimeout(const rclcpp::Time & now) { last_cmd_time_ = now; }
  std::unique_ptr<geometry_msgs::msg::TransformStamped> getTransform(const geometry_msgs::msg::Quaternion &odom_quat, const rclcpp::Time & now);
  std::unique_ptr<nav_msgs::msg::Odometry> getOdometry(const geometry_msgs::msg::Quaternion &odom_quat, const ecl::linear_algebra::Vector3d &pose_update_rates, const rclcpp::Time & now);

private:
  ecl::LegacyPose2D<double> pose_;
  rclcpp::Duration cmd_vel_timeout_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  bool use_imu_heading_;
  rclcpp::Time last_cmd_time_;
};

} // namespace kobuki_node

#endif /* KOBUKI_NODE_ODOMETRY_HPP_ */
