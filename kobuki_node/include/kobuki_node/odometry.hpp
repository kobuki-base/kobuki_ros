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

#include <memory>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ecl/geometry.hpp>
#include <ecl/linear_algebra.hpp>

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
  explicit Odometry(double cmd_vel_timeout_sec, const std::string & odom_frame, const std::string & base_frame, bool publish_tf, bool use_imu_heading, const rclcpp::Time & now);
  Odometry(Odometry && c) = delete;
  Odometry & operator=(Odometry && c) = delete;
  Odometry(const Odometry & c) = delete;
  Odometry & operator=(const Odometry & c) = delete;

  bool commandTimeout(const rclcpp::Time & now) const;
  void update(
    const ecl::linear_algebra::Vector3d &pose_update,
    ecl::linear_algebra::Vector3d &pose_update_rates,
    double imu_heading,
    double imu_angular_velocity,
    const rclcpp::Time & now
  );
  void resetOdometry();
  const rclcpp::Duration& timeout() const;
  void resetTimeout(const rclcpp::Time & now);
  std::unique_ptr<geometry_msgs::msg::TransformStamped> getTransform();
  std::unique_ptr<nav_msgs::msg::Odometry> getOdometry();

private:
  geometry_msgs::msg::Quaternion odom_quat_;
  rclcpp::Time odom_quat_time_;
  ecl::linear_algebra::Vector3d pose_;  // x, y, heading
  ecl::linear_algebra::Vector3d pose_update_rates_;
  rclcpp::Duration cmd_vel_timeout_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  bool use_imu_heading_;
  rclcpp::Time last_cmd_time_;
};

} // namespace kobuki_node

#endif /* KOBUKI_NODE_ODOMETRY_HPP_ */
