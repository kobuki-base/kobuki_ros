/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file /kobuki_safety_controller/src/safety_controller.cpp
 *
 * @brief Implementation for Kobuki's safety controller
 *
 * @author Marcus Liebhardt, Yujin Robot
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <chrono>
#include <functional>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/empty.hpp>

#include <kobuki_ros_interfaces/msg/bumper_event.hpp>
#include <kobuki_ros_interfaces/msg/cliff_event.hpp>
#include <kobuki_ros_interfaces/msg/wheel_drop_event.hpp>

#include "kobuki_safety_controller/safety_controller.hpp"


namespace kobuki_safety_controller
{

SafetyController::SafetyController(const rclcpp::NodeOptions & options) :
  rclcpp::Node("kobuki_safety_controller_node", options),
  controller_active_(false),
  wheel_left_dropped_(false),
  wheel_right_dropped_(false),
  bumper_left_pressed_(false),
  bumper_center_pressed_(false),
  bumper_right_pressed_(false),
  cliff_left_detected_(false),
  cliff_center_detected_(false),
  cliff_right_detected_(false),
  last_event_time_(this->get_clock()->now())
{
  // how long to keep sending messages after a bump, cliff, or wheel drop stops
  this->declare_parameter("time_to_extend_bump_cliff_events", 0.0);

  enable_controller_subscriber_ = this->create_subscription<std_msgs::msg::Empty>("enable", rclcpp::QoS(10), std::bind(&SafetyController::enableCB, this, std::placeholders::_1));
  disable_controller_subscriber_ = this->create_subscription<std_msgs::msg::Empty>("disable", rclcpp::QoS(10), std::bind(&SafetyController::disableCB, this, std::placeholders::_1));
  bumper_event_subscriber_ = this->create_subscription<kobuki_ros_interfaces::msg::BumperEvent>("events/bumper", rclcpp::QoS(10), std::bind(&SafetyController::bumperEventCB, this, std::placeholders::_1));
  cliff_event_subscriber_  = this->create_subscription<kobuki_ros_interfaces::msg::CliffEvent>("events/cliff",  rclcpp::QoS(10), std::bind(&SafetyController::cliffEventCB, this, std::placeholders::_1));
  wheel_event_subscriber_  = this->create_subscription<kobuki_ros_interfaces::msg::WheelDropEvent>("events/wheel_drop", rclcpp::QoS(10), std::bind(&SafetyController::wheelEventCB, this, std::placeholders::_1));
  reset_safety_states_subscriber_ = this->create_subscription<std_msgs::msg::Empty>("reset", rclcpp::QoS(10), std::bind(&SafetyController::resetSafetyStatesCB, this, std::placeholders::_1));
  velocity_command_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  msg_ = std::make_unique<geometry_msgs::msg::Twist>();

  this->enable(); // enable controller

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SafetyController::spin, this));
}

SafetyController:: ~SafetyController()
{
}

bool SafetyController::enable()
{
  if (controller_active_)
  {
    return false;
  }

  controller_active_ = true;
  return true;
}

bool SafetyController::disable()
{
  if (!controller_active_)
  {
    return false;
  }

  controller_active_ = false;
  return true;
}

bool SafetyController::getState()
{
  return controller_active_;
}

void SafetyController::enableCB(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  if (this->enable())
  {
    RCLCPP_INFO(get_logger(), "Controller has been enabled.");
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Controller was already enabled.");
  }
}

void SafetyController::disableCB(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  if (this->disable())
  {
    RCLCPP_INFO(get_logger(), "Controller has been disabled.");
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Controller was already disabled.");
  }
}

void SafetyController::cliffEventCB(const kobuki_ros_interfaces::msg::CliffEvent::SharedPtr msg)
{
  if (msg->state == kobuki_ros_interfaces::msg::CliffEvent::CLIFF)
  {
    last_event_time_ = this->get_clock()->now();
    RCLCPP_DEBUG(get_logger(), "Cliff detected. Moving backwards.");
    switch (msg->sensor)
    {
      case kobuki_ros_interfaces::msg::CliffEvent::LEFT:    cliff_left_detected_   = true;  break;
      case kobuki_ros_interfaces::msg::CliffEvent::CENTER:  cliff_center_detected_ = true;  break;
      case kobuki_ros_interfaces::msg::CliffEvent::RIGHT:   cliff_right_detected_  = true;  break;
    }
  }
  else // kobuki_ros_interfaces::msg::CliffEvent::FLOOR
  {
    RCLCPP_DEBUG(get_logger(), "Not detecting any cliffs. Resuming normal operation.");
    switch (msg->sensor)
    {
      case kobuki_ros_interfaces::msg::CliffEvent::LEFT:    cliff_left_detected_   = false; break;
      case kobuki_ros_interfaces::msg::CliffEvent::CENTER:  cliff_center_detected_ = false; break;
      case kobuki_ros_interfaces::msg::CliffEvent::RIGHT:   cliff_right_detected_  = false; break;
    }
  }
}

void SafetyController::bumperEventCB(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg)
{
  if (msg->state == kobuki_ros_interfaces::msg::BumperEvent::PRESSED)
  {
    last_event_time_ = this->get_clock()->now();
    RCLCPP_DEBUG(get_logger(), "Bumper pressed. Moving backwards.");
    switch (msg->bumper)
    {
      case kobuki_ros_interfaces::msg::BumperEvent::LEFT:    bumper_left_pressed_   = true;  break;
      case kobuki_ros_interfaces::msg::BumperEvent::CENTER:  bumper_center_pressed_ = true;  break;
      case kobuki_ros_interfaces::msg::BumperEvent::RIGHT:   bumper_right_pressed_  = true;  break;
    }
  }
  else // kobuki_ros_interfaces::msg::BumperEvent::RELEASED
  {
    RCLCPP_DEBUG(get_logger(), "No bumper pressed. Resuming normal operation.");
    switch (msg->bumper)
    {
      case kobuki_ros_interfaces::msg::BumperEvent::LEFT:    bumper_left_pressed_   = false; break;
      case kobuki_ros_interfaces::msg::BumperEvent::CENTER:  bumper_center_pressed_ = false; break;
      case kobuki_ros_interfaces::msg::BumperEvent::RIGHT:   bumper_right_pressed_  = false; break;
    }
  }
}

void SafetyController::wheelEventCB(const kobuki_ros_interfaces::msg::WheelDropEvent::SharedPtr msg)
{
  if (msg->state == kobuki_ros_interfaces::msg::WheelDropEvent::DROPPED)
  {
    // need to keep track of both wheels separately
    if (msg->wheel == kobuki_ros_interfaces::msg::WheelDropEvent::LEFT)
    {
      RCLCPP_DEBUG(get_logger(), "Left wheel dropped.");
      wheel_left_dropped_ = true;
    }
    else // kobuki_ros_interfaces::msg::WheelDropEvent::RIGHT
    {
      RCLCPP_DEBUG(get_logger(), "Right wheel dropped.");
      wheel_right_dropped_ = true;
    }
  }
  else // kobuki_ros_interfaces::msg::WheelDropEvent::RAISED
  {
    // need to keep track of both wheels separately
    if (msg->wheel == kobuki_ros_interfaces::msg::WheelDropEvent::LEFT)
    {
      RCLCPP_DEBUG(get_logger(), "Left wheel raised.");
      wheel_left_dropped_ = false;
    }
    else // kobuki_ros_interfaces::msg::WheelDropEvent::RIGHT
    {
      RCLCPP_DEBUG(get_logger(), "Right wheel raised.");
      wheel_right_dropped_ = false;
    }
    if (!wheel_left_dropped_ && !wheel_right_dropped_)
    {
      RCLCPP_DEBUG(get_logger(), "Both wheels raised. Resuming normal operation.");
    }
  }
}

void SafetyController::resetSafetyStatesCB(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  wheel_left_dropped_    = false;
  wheel_right_dropped_   = false;
  bumper_left_pressed_   = false;
  bumper_center_pressed_ = false;
  bumper_right_pressed_  = false;
  cliff_left_detected_   = false;
  cliff_center_detected_ = false;
  cliff_right_detected_  = false;
  RCLCPP_WARN(get_logger(), "All safety states have been reset to false.");
}

void SafetyController::spin()
{
  if (this->getState())
  {
    if (wheel_left_dropped_ || wheel_right_dropped_)
    {
      msg_ = std::make_unique<geometry_msgs::msg::Twist>();
      msg_->linear.x = 0.0;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = 0.0;
      velocity_command_publisher_->publish(*msg_);
    }
    else if (bumper_center_pressed_ || cliff_center_detected_)
    {
      msg_ = std::make_unique<geometry_msgs::msg::Twist>();
      msg_->linear.x = -0.1;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = 0.0;
      velocity_command_publisher_->publish(*msg_);
    }
    else if (bumper_left_pressed_ || cliff_left_detected_)
    {
      // left bump/cliff; also spin a bit to the right to make escape easier
      msg_ = std::make_unique<geometry_msgs::msg::Twist>();
      msg_->linear.x = -0.1;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = -0.4;
      velocity_command_publisher_->publish(*msg_);
    }
    else if (bumper_right_pressed_ || cliff_right_detected_)
    {
      // right bump/cliff; also spin a bit to the left to make escape easier
      msg_ = std::make_unique<geometry_msgs::msg::Twist>();
      msg_->linear.x = -0.1;
      msg_->linear.y = 0.0;
      msg_->linear.z = 0.0;
      msg_->angular.x = 0.0;
      msg_->angular.y = 0.0;
      msg_->angular.z = 0.4;
      velocity_command_publisher_->publish(*msg_);
    }
    else
    {
      double time_to_extend_bump_cliff_events = this->get_parameter("time_to_extend_bump_cliff_events").get_value<double>();
      rclcpp::Duration extend_bump_cliff_events_duration = rclcpp::Duration::from_seconds(time_to_extend_bump_cliff_events);
      // if we want to extend the safety state and we're within the time, just keep sending msg_
      // TODO(clalancette): this is buggy in the case that time_to_extend_bump_cliff_events
      // is set to something > 0.  The very first time through this loop, we could end up
      // publishing an empty msg_ for no reason, since last_event_time ~= this->get_clock()->now().
      // Need to think about how to improve this.
      if (extend_bump_cliff_events_duration > rclcpp::Duration::from_seconds(1e-10) &&
          this->get_clock()->now() - last_event_time_ < extend_bump_cliff_events_duration)
      {
        velocity_command_publisher_->publish(*msg_);
      }
    }
  }
}

} // namespace kobuki_safety_controller

RCLCPP_COMPONENTS_REGISTER_NODE(kobuki_safety_controller::SafetyController)
