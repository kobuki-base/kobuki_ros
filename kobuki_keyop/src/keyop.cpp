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
 * @file /src/keyop.cpp
 *
 * @brief Creates a node for remote controlling parts of robot_core.
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <kobuki_ros_interfaces/msg/keyboard_input.hpp>
#include <kobuki_ros_interfaces/msg/motor_power.hpp>

#include "kobuki_keyop/keyop.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki_keyop
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

/**
 * @brief Default constructor, needs initialisation.
 */
KeyOp::KeyOp(const rclcpp::NodeOptions & options) : rclcpp::Node("kobuki_keyop_node", options),
                         last_zero_vel_sent_(true), // avoid zero-vel messages from the beginning
                         power_status_(false),
                         quit_requested_(false),
                         key_file_descriptor_(0)
{
  tcgetattr(key_file_descriptor_, &original_terminal_state_); // get terminal properties
  cmd_ = std::make_shared<geometry_msgs::msg::Twist>();

  /*********************
   ** Parameters
   **********************/
  double linear_vel_step = this->declare_parameter("linear_vel_step", 0.1);
  double linear_vel_max = this->declare_parameter("linear_vel_max", 3.4);
  double angular_vel_step = this->declare_parameter("angular_vel_step", 0.02);
  double angular_vel_max = this->declare_parameter("angular_vel_max", 1.2);

  RCLCPP_INFO(get_logger(), "KeyOp : using linear  vel step [%f].", linear_vel_step);
  RCLCPP_INFO(get_logger(), "KeyOp : using linear  vel max  [%f].", linear_vel_max);
  RCLCPP_INFO(get_logger(), "KeyOp : using angular vel step [%f].", angular_vel_step);
  RCLCPP_INFO(get_logger(), "KeyOp : using angular vel max  [%f].", angular_vel_max);

  /*********************
   ** Subscribers
   **********************/
  keyinput_subscriber_ = this->create_subscription<kobuki_ros_interfaces::msg::KeyboardInput>("teleop", rclcpp::QoS(1), std::bind(&KeyOp::remoteKeyInputReceived, this, std::placeholders::_1));

  /*********************
   ** Publishers
   **********************/
  velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  motor_power_publisher_ = this->create_publisher<kobuki_ros_interfaces::msg::MotorPower>("motor_power", rclcpp::QoS(1).transient_local());

  auto power_cmd = std::make_unique<kobuki_ros_interfaces::msg::MotorPower>();
  power_cmd->state = kobuki_ros_interfaces::msg::MotorPower::ON;
  motor_power_publisher_->publish(std::move(power_cmd));
  RCLCPP_INFO(get_logger(), "KeyOp: connected.");
  power_status_ = true;

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&KeyOp::spin, this));

  // start keyboard input thread
  thread_ = std::thread(&KeyOp::keyboardInputLoop, this);
}

KeyOp::~KeyOp()
{
  disable();
  quit_requested_ = true;
  thread_.join();
  tcsetattr(key_file_descriptor_, TCSANOW, &original_terminal_state_);
}

/*****************************************************************************
 ** Implementation [Spin]
 *****************************************************************************/

/**
 * @brief Worker thread loop; sends current velocity command at a fixed rate.
 *
 * It also process ros functions as well as aborting when requested.
 */
void KeyOp::spin()
{
  std::lock_guard<std::mutex> lk(cmd_mutex_);

  // Avoid spamming robot with continuous zero-velocity messages
  if ((cmd_->linear.x  != 0.0) || (cmd_->linear.y  != 0.0) || (cmd_->linear.z  != 0.0) ||
      (cmd_->angular.x != 0.0) || (cmd_->angular.y != 0.0) || (cmd_->angular.z != 0.0))
  {
    velocity_publisher_->publish(*cmd_);
    last_zero_vel_sent_ = false;
  }
  else if (last_zero_vel_sent_ == false)
  {
    velocity_publisher_->publish(*cmd_);
    last_zero_vel_sent_ = true;
  }
}

/*****************************************************************************
 ** Implementation [Keyboard]
 *****************************************************************************/

/**
 * @brief The worker thread function that accepts input keyboard commands.
 *
 * This is ok here - but later it might be a good idea to make a node which
 * posts keyboard events to a topic. Recycle common code if used by many!
 */
void KeyOp::keyboardInputLoop()
{
  struct termios raw;
  std::memcpy(&raw, &original_terminal_state_, sizeof(struct termios));

  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(key_file_descriptor_, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Forward/back arrows : linear velocity incr/decr.");
  puts("Right/left arrows : angular velocity incr/decr.");
  puts("Spacebar : reset linear/angular velocities.");
  puts("d : disable motors.");
  puts("e : enable motors.");

  while (!quit_requested_)
  {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(key_file_descriptor_, &readfds);
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 50 * 1000;  // wake up every 50 milliseconds
    int select_ret = ::select(key_file_descriptor_ + 1, &readfds, nullptr, nullptr, &timeout);
    if (select_ret < 0)
    {
      // An error happened
      RCLCPP_ERROR(get_logger(), "Select failed: %s", ::strerror(errno));
    }
    else if (select_ret == 0)
    {
      // timeout occurred, do nothing
    }
    else
    {
      if (!FD_ISSET(key_file_descriptor_, &readfds))
      {
        // We didn't see the file descriptor we expected; error
        RCLCPP_ERROR(get_logger(), "Invalid file descriptor, ignoring");
      }
      char c;
      if (::read(key_file_descriptor_, &c, 1) < 0)
      {
        // The read failed; error
        RCLCPP_ERROR(get_logger(), "Failed to read character: %s", ::strerror(errno));
      }
      else
      {
        processKeyboardInput(c);
      }
    }
  }
}

/**
 * @brief Callback function for remote keyboard inputs subscriber.
 */
void KeyOp::remoteKeyInputReceived(const std::shared_ptr<kobuki_ros_interfaces::msg::KeyboardInput> key)
{
  processKeyboardInput(key->pressed_key);
}

/**
 * @brief Process individual keyboard inputs.
 *
 * @param c keyboard input.
 */
void KeyOp::processKeyboardInput(char c)
{
  // This lock protects against concurrent running of this method (since it can
  // be called either through the keyboard or through the ROS 2 subscription).
  std::lock_guard<std::mutex> lk(cmd_mutex_);

  /*
   * Arrow keys are a bit special, they are escape characters - meaning they
   * trigger a sequence of keycodes. In this case, 'esc-[-Keycode_xxx'. We
   * ignore the esc-[ and just parse the last one. So long as we avoid using
   * the last one for its actual purpose (e.g. left arrow corresponds to
   * esc-[-D) we can keep the parsing simple.
   */
  switch (c)
  {
    case kobuki_ros_interfaces::msg::KeyboardInput::KEYCODE_LEFT:
    {
      incrementAngularVelocity();
      break;
    }
    case kobuki_ros_interfaces::msg::KeyboardInput::KEYCODE_RIGHT:
    {
      decrementAngularVelocity();
      break;
    }
    case kobuki_ros_interfaces::msg::KeyboardInput::KEYCODE_UP:
    {
      incrementLinearVelocity();
      break;
    }
    case kobuki_ros_interfaces::msg::KeyboardInput::KEYCODE_DOWN:
    {
      decrementLinearVelocity();
      break;
    }
    case kobuki_ros_interfaces::msg::KeyboardInput::KEYCODE_SPACE:
    {
      resetVelocity();
      break;
    }
    case 'd':
    {
      disable();
      break;
    }
    case 'e':
    {
      enable();
      break;
    }
    default:
    {
      break;
    }
  }
}

/*****************************************************************************
 ** Implementation [Commands]
 *****************************************************************************/
/**
 * @brief Disables commands to the navigation system.
 *
 * This does the following things:
 *
 * - Disables power to the navigation motors (via device_manager).
 * @param msg
 */
void KeyOp::disable()
{
  cmd_->linear.x = 0.0;
  cmd_->angular.z = 0.0;
  velocity_publisher_->publish(*cmd_);

  if (power_status_)
  {
    RCLCPP_INFO(get_logger(), "KeyOp: die, die, die (disabling power to the device's motor system).");
    auto power_cmd = std::make_unique<kobuki_ros_interfaces::msg::MotorPower>();
    power_cmd->state = kobuki_ros_interfaces::msg::MotorPower::OFF;
    motor_power_publisher_->publish(std::move(power_cmd));
    power_status_ = false;
  }
  else
  {
    RCLCPP_WARN(get_logger(), "KeyOp: Motor system has already been powered down.");
  }
}

/**
 * @brief Reset/re-enable the navigation system.
 *
 * - resets the command velocities.
 * - reenable power if not enabled.
 */
void KeyOp::enable()
{
  cmd_->linear.x = 0.0;
  cmd_->angular.z = 0.0;
  velocity_publisher_->publish(*cmd_);

  if (!power_status_)
  {
    RCLCPP_INFO(get_logger(), "KeyOp: Enabling power to the device subsystem.");
    auto power_cmd = std::make_unique<kobuki_ros_interfaces::msg::MotorPower>();
    power_cmd->state = kobuki_ros_interfaces::msg::MotorPower::ON;
    motor_power_publisher_->publish(std::move(power_cmd));
    power_status_ = true;
  }
  else
  {
    RCLCPP_WARN(get_logger(), "KeyOp: Device has already been powered up.");
  }
}

/**
 * @brief If not already maxxed, increment the command velocities..
 */
void KeyOp::incrementLinearVelocity()
{
  if (power_status_)
  {
    double linear_vel_max = get_parameter("linear_vel_max").get_value<double>();
    double linear_vel_step = get_parameter("linear_vel_step").get_value<double>();
    if (cmd_->linear.x <= linear_vel_max)
    {
      cmd_->linear.x += linear_vel_step;
    }
    RCLCPP_INFO(get_logger(), "KeyOp: linear  velocity incremented [%f|%f]", cmd_->linear.x, cmd_->angular.z);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "KeyOp: motors are not yet powered up.");
  }
}

/**
 * @brief If not already minned, decrement the linear velocities..
 */
void KeyOp::decrementLinearVelocity()
{
  if (power_status_)
  {
    double linear_vel_max = get_parameter("linear_vel_max").get_value<double>();
    double linear_vel_step = get_parameter("linear_vel_step").get_value<double>();
    if (cmd_->linear.x >= -linear_vel_max)
    {
      cmd_->linear.x -= linear_vel_step;
    }
    RCLCPP_INFO(get_logger(), "KeyOp: linear  velocity decremented [%f|%f]", cmd_->linear.x, cmd_->angular.z);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "KeyOp: motors are not yet powered up.");
  }
}

/**
 * @brief If not already maxxed, increment the angular velocities..
 */
void KeyOp::incrementAngularVelocity()
{
  if (power_status_)
  {
    double angular_vel_max = get_parameter("angular_vel_max").get_value<double>();
    double angular_vel_step = get_parameter("angular_vel_step").get_value<double>();
    if (cmd_->angular.z <= angular_vel_max)
    {
      cmd_->angular.z += angular_vel_step;
    }
    RCLCPP_INFO(get_logger(), "KeyOp: angular velocity incremented [%f|%f]", cmd_->linear.x, cmd_->angular.z);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "KeyOp: motors are not yet powered up.");
  }
}

/**
 * @brief If not already mined, decrement the angular velocities..
 */
void KeyOp::decrementAngularVelocity()
{
  if (power_status_)
  {
    double angular_vel_max = get_parameter("angular_vel_max").get_value<double>();
    double angular_vel_step = get_parameter("angular_vel_step").get_value<double>();
    if (cmd_->angular.z >= -angular_vel_max)
    {
      cmd_->angular.z -= angular_vel_step;
    }
    RCLCPP_INFO(get_logger(), "KeyOp: angular velocity decremented [%f|%f]", cmd_->linear.x, cmd_->angular.z);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "KeyOp: motors are not yet powered up.");
  }
}

void KeyOp::resetVelocity()
{
  if (power_status_)
  {
    cmd_->angular.z = 0.0;
    cmd_->linear.x = 0.0;
    RCLCPP_INFO(get_logger(), "KeyOp: reset linear/angular velocities.");
  }
  else
  {
    RCLCPP_WARN(get_logger(), "KeyOp: motors are not yet powered up.");
  }
}

} // namespace kobuki_keyop

RCLCPP_COMPONENTS_REGISTER_NODE(kobuki_keyop::KeyOp)
