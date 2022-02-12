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
 * @file /include/kobuki_keyop/keyop.hpp
 *
 * @brief The controlling node for remote operations on robot_core.
 *
 **/

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef KOBUKI_KEYOP_HPP_
#define KOBUKI_KEYOP_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <termios.h> // for keyboard input

#include <memory>
#include <mutex>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>  // for velocity commands
#include <rclcpp/rclcpp.hpp>

#include <kobuki_ros_interfaces/msg/keyboard_input.hpp> // keycodes from remote teleops.
#include <kobuki_ros_interfaces/msg/motor_power.hpp>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki_keyop
{

/*****************************************************************************
 ** Interface
 *****************************************************************************/
/**
 * @brief Keyboard remote control for our robot core (mobile base).
 *
 */
class KeyOp final : public rclcpp::Node
{
public:
  /*********************
   ** C&D
   **********************/
  explicit KeyOp(const rclcpp::NodeOptions & options);
  ~KeyOp() override;
  KeyOp(KeyOp && c) = delete;
  KeyOp & operator=(KeyOp && c) = delete;
  KeyOp(const KeyOp & c) = delete;
  KeyOp & operator=(const KeyOp & c) = delete;

private:
  rclcpp::Subscription<kobuki_ros_interfaces::msg::KeyboardInput>::SharedPtr keyinput_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::MotorPower>::SharedPtr motor_power_publisher_;
  bool last_zero_vel_sent_;
  bool power_status_;
  // protects against concurrent access to cmd_ (and friends) by the thread
  // dealing with keyboard strokes and remote keyboard strokes
  std::mutex cmd_mutex_;
  std::shared_ptr<geometry_msgs::msg::Twist> cmd_;
  rclcpp::TimerBase::SharedPtr timer_;

  /*********************
   ** Runtime
   **********************/
  void spin();

  /*********************
   ** Commands
   **********************/
  void enable();
  void disable();
  void incrementLinearVelocity();
  void decrementLinearVelocity();
  void incrementAngularVelocity();
  void decrementAngularVelocity();
  void resetVelocity();

  /*********************
   ** Keylogging
   **********************/

  void keyboardInputLoop();
  void processKeyboardInput(char c);
  void remoteKeyInputReceived(const std::shared_ptr<kobuki_ros_interfaces::msg::KeyboardInput> key);
  void restoreTerminal();
  bool quit_requested_;
  int key_file_descriptor_;
  struct termios original_terminal_state_;
  std::thread thread_;
};

} // namespace kobuki_keyop

#endif /* KOBUKI_KEYOP_HPP_ */
