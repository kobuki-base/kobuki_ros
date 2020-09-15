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
 * @file /kobuki_node/include/kobuki_node/kobuki_ros.hpp
 *
 * @brief Wraps the kobuki driver in a ROS-specific library
 *
 **/

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef KOBUKI_ROS_HPP_
#define KOBUKI_ROS_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <ecl/sigslots.hpp>

#include <kobuki_ros_interfaces/msg/button_event.hpp>
#include <kobuki_ros_interfaces/msg/bumper_event.hpp>
#include <kobuki_ros_interfaces/msg/cliff_event.hpp>
#include <kobuki_ros_interfaces/msg/controller_info.hpp>
#include <kobuki_ros_interfaces/msg/digital_output.hpp>
#include <kobuki_ros_interfaces/msg/digital_input_event.hpp>
#include <kobuki_ros_interfaces/msg/external_power.hpp>
#include <kobuki_ros_interfaces/msg/dock_infra_red.hpp>
#include <kobuki_ros_interfaces/msg/led.hpp>
#include <kobuki_ros_interfaces/msg/motor_power.hpp>
#include <kobuki_ros_interfaces/msg/power_system_event.hpp>
#include <kobuki_ros_interfaces/msg/robot_state_event.hpp>
#include <kobuki_ros_interfaces/msg/sensor_state.hpp>
#include <kobuki_ros_interfaces/msg/sound.hpp>
#include <kobuki_ros_interfaces/msg/version_info.hpp>
#include <kobuki_ros_interfaces/msg/wheel_drop_event.hpp>

#include <kobuki_core/kobuki.hpp>

#include "diagnostics.hpp"
#include "odometry.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki_node
{
class KobukiRos final : public rclcpp::Node
{
public:
  explicit KobukiRos(const rclcpp::NodeOptions & options);
  ~KobukiRos() override;
  KobukiRos(KobukiRos && c) = delete;
  KobukiRos & operator=(KobukiRos && c) = delete;
  KobukiRos(const KobukiRos & c) = delete;
  KobukiRos & operator=(const KobukiRos & c) = delete;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  void update();

  /*********************
   ** Variables
   **********************/
  kobuki::Kobuki kobuki_;
  sensor_msgs::msg::JointState joint_states_;
  std::unique_ptr<Odometry> odometry_;
  bool cmd_vel_timed_out_; // stops warning spam when cmd_vel flags as timed out more than once in a row
  bool serial_timed_out_; // stops warning spam when serial connection timed out more than once in a row
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

  /*********************
   ** Ros Comms
   **********************/
  rclcpp::Publisher<kobuki_ros_interfaces::msg::VersionInfo>::SharedPtr version_info_publisher_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::ControllerInfo>::SharedPtr controller_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::SensorState>::SharedPtr sensor_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::DockInfraRed>::SharedPtr dock_ir_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_data_publisher_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_event_publisher_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::DigitalInputEvent>::SharedPtr input_event_publisher_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::RobotStateEvent>::SharedPtr robot_event_publisher_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_event_publisher_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::CliffEvent>::SharedPtr cliff_event_publisher_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::WheelDropEvent>::SharedPtr wheel_event_publisher_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::PowerSystemEvent>::SharedPtr power_event_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_data_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_data_stream_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr raw_control_command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_command_subscriber_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::DigitalOutput>::SharedPtr digital_output_command_subscriber_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::ExternalPower>::SharedPtr external_power_command_subscriber_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::ControllerInfo>::SharedPtr controller_info_command_subscriber_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::Led>::SharedPtr led1_command_subscriber_, led2_command_subscriber_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::Sound>::SharedPtr sound_command_subscriber_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::MotorPower>::SharedPtr motor_power_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_odometry_subscriber_;

  rclcpp::TimerBase::SharedPtr timer_;

  /*********************
  ** Ros Callbacks
  **********************/
  void subscribeVelocityCommand(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
  void subscribeLed1Command(const std::shared_ptr<kobuki_ros_interfaces::msg::Led> msg);
  void subscribeLed2Command(const std::shared_ptr<kobuki_ros_interfaces::msg::Led> msg);
  void subscribeDigitalOutputCommand(const std::shared_ptr<kobuki_ros_interfaces::msg::DigitalOutput> msg);
  void subscribeExternalPowerCommand(const std::shared_ptr<kobuki_ros_interfaces::msg::ExternalPower> msg);
  void subscribeResetOdometry(const std::shared_ptr<std_msgs::msg::Empty> msg);
  void subscribeSoundCommand(const std::shared_ptr<kobuki_ros_interfaces::msg::Sound> msg);
  void subscribeMotorPower(const std::shared_ptr<kobuki_ros_interfaces::msg::MotorPower> msg);
  void subscribeControllerInfoCommand(const std::shared_ptr<kobuki_ros_interfaces::msg::ControllerInfo> msg);

  /*********************
   ** SigSlots
   **********************/
  ecl::Slot<const kobuki::VersionInfo&> slot_version_info_;
  ecl::Slot<> slot_stream_data_;
  ecl::Slot<> slot_controller_info_;
  ecl::Slot<const kobuki::ButtonEvent&> slot_button_event_;
  ecl::Slot<const kobuki::BumperEvent&> slot_bumper_event_;
  ecl::Slot<const kobuki::CliffEvent&>  slot_cliff_event_;
  ecl::Slot<const kobuki::WheelEvent&>  slot_wheel_event_;
  ecl::Slot<const kobuki::PowerEvent&>  slot_power_event_;
  ecl::Slot<const kobuki::InputEvent&>  slot_input_event_;
  ecl::Slot<const kobuki::RobotEvent&>  slot_robot_event_;
  ecl::Slot<const std::string&> slot_debug_, slot_info_, slot_warn_, slot_error_;
  ecl::Slot<kobuki::Command::Buffer&> slot_raw_data_command_;
  ecl::Slot<kobuki::PacketFinder::BufferType&> slot_raw_data_stream_;
  ecl::Slot<const std::vector<short>&> slot_raw_control_command_;

  /*********************
   ** Slot Callbacks
   **********************/
  void processStreamData();
  void publishWheelState();
  void publishInertia();
  void publishRawInertia();
  void publishSensorState();
  void publishDockIRData();
  void publishBatteryState();
  void publishVersionInfo(const kobuki::VersionInfo &version_info);
  void publishControllerInfo();
  void publishButtonEvent(const kobuki::ButtonEvent &event);
  void publishBumperEvent(const kobuki::BumperEvent &event);
  void publishCliffEvent(const kobuki::CliffEvent &event);
  void publishWheelEvent(const kobuki::WheelEvent &event);
  void publishPowerEvent(const kobuki::PowerEvent &event);
  void publishInputEvent(const kobuki::InputEvent &event);
  void publishRobotEvent(const kobuki::RobotEvent &event);


  // debugging
  void rosDebug(const std::string &msg) { RCLCPP_DEBUG(get_logger(), "%s", msg.c_str()); }
  void rosInfo(const std::string &msg) { RCLCPP_INFO(get_logger(), "%s", msg.c_str()); }
  void rosWarn(const std::string &msg) { RCLCPP_WARN(get_logger(), "%s", msg.c_str()); }
  void rosError(const std::string &msg) { RCLCPP_ERROR(get_logger(), "%s", msg.c_str()); }

  void publishRawDataCommand(kobuki::Command::Buffer &buffer);
  void publishRawDataStream(kobuki::PacketFinder::BufferType &buffer);
  void publishRawControlCommand(const std::vector<short> &velocity_commands);

  /*********************
  ** Diagnostics
  **********************/
  diagnostic_updater::Updater updater_;
  BatteryTask     battery_diagnostics_;
  WatchdogTask   watchdog_diagnostics_;
  CliffSensorTask   cliff_diagnostics_;
  WallSensorTask   bumper_diagnostics_;
  WheelDropTask     wheel_diagnostics_;
  MotorCurrentTask  motor_diagnostics_;
  MotorStateTask    state_diagnostics_;
  GyroSensorTask     gyro_diagnostics_;
  DigitalInputTask dinput_diagnostics_;
  AnalogInputTask  ainput_diagnostics_;
};

} // namespace kobuki_node

#endif /* KOBUKI_ROS_HPP_ */
