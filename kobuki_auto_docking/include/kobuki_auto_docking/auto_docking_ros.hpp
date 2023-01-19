/**
 * @file /auto_docking/include/auto_docking/auto_docking_ros.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef AUTO_DOCKING_ROS_HPP_
#define AUTO_DOCKING_ROS_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <kobuki_ros_interfaces/action/auto_docking.hpp>
#include <kobuki_ros_interfaces/msg/sensor_state.hpp>
#include <kobuki_ros_interfaces/msg/dock_infra_red.hpp>

#include <kobuki_core/dock_drive.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace kobuki_auto_docking
{

typedef message_filters::sync_policies::ApproximateTime<
  nav_msgs::msg::Odometry,
  kobuki_ros_interfaces::msg::SensorState,
  kobuki_ros_interfaces::msg::DockInfraRed
> SyncPolicy;

class AutoDockingROS final : public rclcpp::Node
{
public:
  AutoDockingROS(const rclcpp::NodeOptions & options);
  ~AutoDockingROS();

private:
  using AutoDocking = kobuki_ros_interfaces::action::AutoDocking;
  using GoalHandleAutoDocking = rclcpp_action::ServerGoalHandle<AutoDocking>;

  kobuki::DockDrive dock_;

  rclcpp_action::Server<kobuki_ros_interfaces::action::AutoDocking>::SharedPtr as_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_commander_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_jabber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr debug_;

  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;
  std::shared_ptr<message_filters::Subscriber<kobuki_ros_interfaces::msg::DockInfraRed>> ir_sub_;
  std::shared_ptr<message_filters::Subscriber<kobuki_ros_interfaces::msg::SensorState>> core_sub_;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  std::shared_ptr<rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>> parameter_subscription_;

  void syncCb(
    const std::shared_ptr<nav_msgs::msg::Odometry> odom,
    const std::shared_ptr<kobuki_ros_interfaces::msg::SensorState> core,
    const std::shared_ptr<kobuki_ros_interfaces::msg::DockInfraRed> ir);

  void debugCb(const std::shared_ptr<std_msgs::msg::String> msg);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const AutoDocking::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleAutoDocking> goal_handle);

  void handle_accepted(
    const std::shared_ptr<GoalHandleAutoDocking> goal_handle);

  void execute(
    const std::shared_ptr<GoalHandleAutoDocking> goal_handle);

  void on_parameter_event(
    std::shared_ptr<rcl_interfaces::msg::ParameterEvent> event);
};

}  // namespace kobuki_auto_docking

#endif /* AUTO_DOCKING_ROS_HPP_ */
