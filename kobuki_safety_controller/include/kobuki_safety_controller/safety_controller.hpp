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
 * @file /kobuki_safety_controller/include/kobuki_safety_controller/safety_controller.hpp
 *
 * @brief Kobuki-specific safety controller
 *
 * This controller uses Kobuki's bumper, cliff and wheel drop sensors to ensure safe operation.
 *
 * @author Marcus Liebhardt, Yujin Robot
 *
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef SAFETY_CONTROLLER_HPP_
#define SAFETY_CONTROLLER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <kobuki_ros_interfaces/msg/bumper_event.hpp>
#include <kobuki_ros_interfaces/msg/cliff_event.hpp>
#include <kobuki_ros_interfaces/msg/wheel_drop_event.hpp>

namespace kobuki_safety_controller
{

/**
 * @ brief Keeps track of safety-related events and commands Kobuki to move accordingly
 *
 * The SafetyController keeps track of bumper, cliff and wheel drop events. In case of the first two,
 * Kobuki is commanded to move back. In the latter case, Kobuki is stopped. All commands stop when the
 * event condition disappears. In the case of lateral bump/cliff, robot also spins a bit, what makes
 * easier to escape from the risk.
 *
 * This controller can be enabled/disabled.
 * The safety states (bumper pressed etc.) can be reset. WARNING: Dangerous!
 */
class SafetyController : public rclcpp::Node
{
public:
  explicit SafetyController(const rclcpp::NodeOptions & options);
  ~SafetyController() override;
  SafetyController(SafetyController && c) = delete;
  SafetyController & operator=(SafetyController && c) = delete;
  SafetyController(const SafetyController & c) = delete;
  SafetyController & operator=(const SafetyController & c) = delete;

private:
  bool controller_active_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr enable_controller_subscriber_, disable_controller_subscriber_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_event_subscriber_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::CliffEvent>::SharedPtr cliff_event_subscriber_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::WheelDropEvent>::SharedPtr wheel_event_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_safety_states_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_command_publisher_;
  bool wheel_left_dropped_, wheel_right_dropped_;
  bool bumper_left_pressed_, bumper_center_pressed_, bumper_right_pressed_;
  bool cliff_left_detected_, cliff_center_detected_, cliff_right_detected_;
  rclcpp::Time last_event_time_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<geometry_msgs::msg::Twist> msg_; // velocity command

  bool enable();

  bool disable();

  bool getState();

  /**
   * @ brief Checks safety states and publishes velocity commands when necessary
   */
  void spin();

  /**
   * @brief ROS logging output for enabling the controller
   * @param msg incoming topic message
   */
  void enableCB(const std_msgs::msg::Empty::SharedPtr msg);

  /**
   * @brief ROS logging output for disabling the controller
   * @param msg incoming topic message
   */
  void disableCB(const std_msgs::msg::Empty::SharedPtr msg);

  /**
   * @brief Keeps track of bumps
   * @param msg incoming topic message
   */
  void bumperEventCB(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg);

  /**
   * @brief Keeps track of cliff detection
   * @param msg incoming topic message
   */
  void cliffEventCB(const kobuki_ros_interfaces::msg::CliffEvent::SharedPtr msg);

  /**
   * @brief Keeps track of the wheel states
   * @param msg incoming topic message
   */
  void wheelEventCB(const kobuki_ros_interfaces::msg::WheelDropEvent::SharedPtr msg);

  /**
   * @brief Callback for resetting safety variables
   *
   * Allows resetting bumper, cliff and wheel drop states.
   * DANGEROUS!
   *
   * @param msg incoming topic message
   */
  void resetSafetyStatesCB(const std_msgs::msg::Empty::SharedPtr msg);
};


} // namespace kobuki_safety_controller

#endif /* SAFETY_CONTROLLER_HPP_ */
