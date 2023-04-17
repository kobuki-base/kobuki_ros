/**
 * @file /auto_docking/src/auto_docking_ros.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/parameter_events_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

#include <ecl/linear_algebra.hpp>

#include <kobuki_ros_interfaces/msg/dock_infra_red.hpp>
#include <kobuki_ros_interfaces/msg/sensor_state.hpp>

#include "kobuki_auto_docking/auto_docking_ros.hpp"

namespace kobuki_auto_docking
{
AutoDockingROS::AutoDockingROS(const rclcpp::NodeOptions & options) : rclcpp::Node("kobuki_auto_docking", options){
  this->as_ = rclcpp_action::create_server<AutoDocking>(
    this,
    "auto_docking_action",
    std::bind(&AutoDockingROS::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&AutoDockingROS::handle_cancel, this, std::placeholders::_1),
    std::bind(&AutoDockingROS::handle_accepted, this, std::placeholders::_1));

  // Configure docking drive
  double min_abs_v = this->declare_parameter<double>("min_abs_v", 0.01);
  double min_abs_w = this->declare_parameter<double>("min_abs_w", 0.1);

  dock_.setMinAbsV(min_abs_v);
  dock_.setMinAbsW(min_abs_w);

  // parameter callback
  parameter_subscription_ = rclcpp::AsyncParametersClient::on_parameter_event(
    this->get_node_topics_interface(),
    std::bind(&AutoDockingROS::on_parameter_event, this, std::placeholders::_1));

  // Publishers and subscribers
  velocity_commander_ = this->create_publisher<geometry_msgs::msg::Twist>("commands/velocity", 10);
  debug_jabber_ = this->create_publisher<std_msgs::msg::String>("debug/feedback", 10);

  debug_ = this->create_subscription<std_msgs::msg::String>(
    "debug/mode_shift", 10, std::bind(&AutoDockingROS::debugCb, this, std::placeholders::_1));

  odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "odom");
  core_sub_ = std::make_shared<message_filters::Subscriber<kobuki_ros_interfaces::msg::SensorState>>(this, "sensors/core");
  ir_sub_ = std::make_shared<message_filters::Subscriber<kobuki_ros_interfaces::msg::DockInfraRed>>(this, "sensors/dock_ir");

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *odom_sub_, *core_sub_, *ir_sub_);
  sync_->registerCallback(&AutoDockingROS::syncCb, this);

  dock_.init();
}

AutoDockingROS::~AutoDockingROS()
{
  if (dock_.isEnabled()) {
    dock_.disable();
  }
}

rclcpp_action::GoalResponse AutoDockingROS::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const AutoDocking::Goal> goal)
{
  (void)uuid;
  (void)goal;

  RCLCPP_INFO(this->get_logger(), "Received goal request");

  if (dock_.isEnabled()) {
    auto result_ = std::make_shared<AutoDocking::Result>();
    result_->text = "Rejected: dock_drive is already enabled.";
    RCLCPP_INFO(this->get_logger(), "New goal received but rejected");

    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse AutoDockingROS::handle_cancel(
  const std::shared_ptr<GoalHandleAutoDocking> goal_handle)
{
  (void)goal_handle;

  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

  if (dock_.isEnabled()) {
    dock_.disable();
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void AutoDockingROS::handle_accepted(
  const std::shared_ptr<GoalHandleAutoDocking> goal_handle)
{
  std::thread{std::bind(&AutoDockingROS::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void AutoDockingROS::execute(
  const std::shared_ptr<GoalHandleAutoDocking> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "New goal received and accepted");

  rclcpp::Rate loop_rate(1);

  auto feedback_ = std::make_shared<AutoDocking::Feedback>();
  auto result_ = std::make_shared<AutoDocking::Result>();

  dock_.enable();

  while ((dock_.getState() != kobuki::RobotDockingState::DONE) && rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "goal_handle->is_canceling()");
      result_->text = "Cancelled: Cancel requested.";
      goal_handle->canceled(result_);
      RCLCPP_INFO(this->get_logger(), "[kobuki_auto_docking] %s", result_->text.c_str());
      dock_.disable();
      return;
    } else if (!dock_.isEnabled()) {  // Action Server is activated, but DockDrive is not enabled, or disabled unexpectedly
      RCLCPP_ERROR(this->get_logger(), "Unintended Case: ActionService is active, but DockDrive is not enabled..");
      result_->text = "Aborted: dock_drive is disabled unexpectedly";
      goal_handle->abort(result_);
      RCLCPP_DEBUG(this->get_logger(), "Goal aborted.");
      dock_.disable();
      return;
    } else {
      feedback_->state = dock_.getStateStr();
      feedback_->text = dock_.getDebugStr();
      goal_handle->publish_feedback(feedback_);
      RCLCPP_DEBUG(this->get_logger(), "Feedback sent");
    }

    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    result_->text = "Arrived on docking station successfully";
    goal_handle->succeed(result_);
    RCLCPP_INFO(this->get_logger(), "Arrived on docking station successfully");
    RCLCPP_DEBUG(this->get_logger(), "Result sent");
    dock_.disable();
  }
}

void AutoDockingROS::syncCb(
  const std::shared_ptr<nav_msgs::msg::Odometry> odom,
  const std::shared_ptr<kobuki_ros_interfaces::msg::SensorState> core,
  const std::shared_ptr<kobuki_ros_interfaces::msg::DockInfraRed> ir)
{
  // process and run
  if (this->dock_.isEnabled()) {

    // conversions
    double roll, pitch, yaw;

    tf2::Quaternion q(
      odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z,
      odom->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);

    ecl::linear_algebra::Vector3d pose;  // x, y, heading
    pose[0] = odom->pose.pose.position.x;
    pose[1] = odom->pose.pose.position.y;
    pose[2] = yaw;

    // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, yaw: %f", pose[0], pose[1], pose[2]);

    // update
    this->dock_.update(ir->data, core->bumper, core->charger, pose);

    // publish debug stream
    auto debug_log = std_msgs::msg::String();
    debug_log.data = this->dock_.getDebugStream();
    debug_jabber_->publish(debug_log);

    // publish command velocity
    if (this->dock_.canRun()) {
      auto cmd_vel = geometry_msgs::msg::Twist();
      cmd_vel.linear.x = this->dock_.getVX();
      cmd_vel.angular.z = this->dock_.getWZ();
      velocity_commander_->publish(cmd_vel);
    }
  }
}

void AutoDockingROS::debugCb(const std::shared_ptr<std_msgs::msg::String> msg)
{
  dock_.modeShift(msg->data);
}

void AutoDockingROS::on_parameter_event(
  std::shared_ptr<rcl_interfaces::msg::ParameterEvent> event)
{
  // Filter out events from other nodes
  if (event->node != this->get_fully_qualified_name()) {
    return;
  }

  // Filter for "min_abs_v" or "min_abs_w" being changed.
  rclcpp::ParameterEventsFilter filter(event, {"min_abs_v", "min_abs_w"}, {rclcpp::ParameterEventsFilter::EventType::CHANGED});
  for (const rclcpp::ParameterEventsFilter::EventPair & it : filter.get_events()) {
    if (it.second->name == "min_abs_v") {
      RCLCPP_INFO(this->get_logger(), "dock_drive update [min_abs_v: %f]", it.second->value.double_value);
      dock_.setMinAbsV(it.second->value.double_value);
    } else if (it.second->name == "min_abs_w") {
      RCLCPP_INFO(this->get_logger(), "dock_drive update [min_abs_w: %f]", it.second->value.double_value);
      dock_.setMinAbsW(it.second->value.double_value);
    }
  }
}

}  // namespace kobuki_auto_docking

RCLCPP_COMPONENTS_REGISTER_NODE(kobuki_auto_docking::AutoDockingROS)
