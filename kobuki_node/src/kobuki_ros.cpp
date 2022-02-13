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
 * @file /kobuki_node/src/node/kobuki_node.cpp
 *
 * @brief Implementation for the ros kobuki node wrapper.
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <chrono>
#include <cfloat>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <angles/angles.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ecl/errors.hpp>
#include <ecl/exceptions.hpp>
#include <ecl/geometry.hpp>

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

#include "kobuki_node/kobuki_ros.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kobuki_node
{

/*****************************************************************************
 ** Implementation [KobukiRos]
 *****************************************************************************/

/**
 * @brief Default constructor.
 *
 * Make sure you call the init() method to fully define this node.
 */
KobukiRos::KobukiRos(const rclcpp::NodeOptions & options) : rclcpp::Node("kobuki", options),
    cmd_vel_timed_out_(false), serial_timed_out_(false),
    slot_version_info_(&KobukiRos::publishVersionInfo, *this),
    slot_stream_data_(&KobukiRos::processStreamData, *this),
    slot_controller_info_(&KobukiRos::publishControllerInfo, *this),
    slot_button_event_(&KobukiRos::publishButtonEvent, *this),
    slot_bumper_event_(&KobukiRos::publishBumperEvent, *this),
    slot_cliff_event_(&KobukiRos::publishCliffEvent, *this),
    slot_wheel_event_(&KobukiRos::publishWheelEvent, *this),
    slot_power_event_(&KobukiRos::publishPowerEvent, *this),
    slot_input_event_(&KobukiRos::publishInputEvent, *this),
    slot_robot_event_(&KobukiRos::publishRobotEvent, *this),
    slot_debug_(&KobukiRos::rosDebug, *this),
    slot_info_(&KobukiRos::rosInfo, *this),
    slot_warn_(&KobukiRos::rosWarn, *this),
    slot_error_(&KobukiRos::rosError, *this),
    slot_raw_data_command_(&KobukiRos::publishRawDataCommand, *this),
    slot_raw_data_stream_(&KobukiRos::publishRawDataStream, *this),
    slot_raw_control_command_(&KobukiRos::publishRawControlCommand, *this),
    updater_(this)
{
  updater_.setHardwareID("Kobuki");
  updater_.add(battery_diagnostics_);
  updater_.add(watchdog_diagnostics_);
  updater_.add(bumper_diagnostics_);
  updater_.add(cliff_diagnostics_);
  updater_.add(wheel_diagnostics_);
  updater_.add(motor_diagnostics_);
  updater_.add(state_diagnostics_);
  updater_.add(gyro_diagnostics_);
  updater_.add(dinput_diagnostics_);
  updater_.add(ainput_diagnostics_);

  /*********************
   ** Communications
   **********************/
  /*********************
  ** Turtlebot Required
  **********************/
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 100);

  /*********************
  ** Kobuki Esoterics
  **********************/
  version_info_publisher_ = this->create_publisher<kobuki_ros_interfaces::msg::VersionInfo>("version_info", rclcpp::QoS(rclcpp::KeepLast(100)).transient_local()); // latched publisher
  controller_info_publisher_ = this->create_publisher<kobuki_ros_interfaces::msg::ControllerInfo>("controller_info", rclcpp::QoS(rclcpp::KeepLast(100)).transient_local()); // latched publisher
  button_event_publisher_ = this->create_publisher<kobuki_ros_interfaces::msg::ButtonEvent>("events/button", 100);
  bumper_event_publisher_ = this->create_publisher<kobuki_ros_interfaces::msg::BumperEvent>("events/bumper", 100);
  cliff_event_publisher_  = this->create_publisher<kobuki_ros_interfaces::msg::CliffEvent>("events/cliff",  100);
  wheel_event_publisher_  = this->create_publisher<kobuki_ros_interfaces::msg::WheelDropEvent>("events/wheel_drop", 100);
  power_event_publisher_  = this->create_publisher<kobuki_ros_interfaces::msg::PowerSystemEvent>("events/power_system", 100);
  input_event_publisher_  = this->create_publisher<kobuki_ros_interfaces::msg::DigitalInputEvent>("events/digital_input", 100);
  robot_event_publisher_  = this->create_publisher<kobuki_ros_interfaces::msg::RobotStateEvent>("events/robot_state", rclcpp::QoS(rclcpp::KeepLast(100)).transient_local()); // also latched
  sensor_state_publisher_ = this->create_publisher<kobuki_ros_interfaces::msg::SensorState>("sensors/core", 100);
  dock_ir_publisher_ = this->create_publisher<kobuki_ros_interfaces::msg::DockInfraRed>("sensors/dock_ir", 100);
  battery_state_publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("sensors/battery_state", 100);
  imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("sensors/imu_data", 100);
  raw_imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("sensors/imu_data_raw", 100);
  raw_data_command_publisher_ = this->create_publisher<std_msgs::msg::String>("debug/raw_data_command", 100);
  raw_data_stream_publisher_ = this->create_publisher<std_msgs::msg::String>("debug/raw_data_stream", 100);
  raw_control_command_publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("debug/raw_control_command", 100);
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50); // topic name and queue size

  odom_broadcaster_= std::make_unique<tf2_ros::TransformBroadcaster>(this);

  velocity_command_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("commands/velocity", rclcpp::QoS(10), std::bind(&KobukiRos::subscribeVelocityCommand, this, std::placeholders::_1));
  led1_command_subscriber_ =  this->create_subscription<kobuki_ros_interfaces::msg::Led>("commands/led1", rclcpp::QoS(10), std::bind(&KobukiRos::subscribeLed1Command, this, std::placeholders::_1));
  led2_command_subscriber_ =  this->create_subscription<kobuki_ros_interfaces::msg::Led>("commands/led2", rclcpp::QoS(10), std::bind(&KobukiRos::subscribeLed2Command, this, std::placeholders::_1));
  digital_output_command_subscriber_ =  this->create_subscription<kobuki_ros_interfaces::msg::DigitalOutput>("commands/digital_output", rclcpp::QoS(10), std::bind(&KobukiRos::subscribeDigitalOutputCommand, this, std::placeholders::_1));
  external_power_command_subscriber_ =  this->create_subscription<kobuki_ros_interfaces::msg::ExternalPower>("commands/external_power", rclcpp::QoS(10), std::bind(&KobukiRos::subscribeExternalPowerCommand, this, std::placeholders::_1));
  sound_command_subscriber_ =  this->create_subscription<kobuki_ros_interfaces::msg::Sound>("commands/sound", rclcpp::QoS(10), std::bind(&KobukiRos::subscribeSoundCommand, this, std::placeholders::_1));
  reset_odometry_subscriber_ = this->create_subscription<std_msgs::msg::Empty>("commands/reset_odometry", rclcpp::QoS(10), std::bind(&KobukiRos::subscribeResetOdometry, this, std::placeholders::_1));
  motor_power_subscriber_ = this->create_subscription<kobuki_ros_interfaces::msg::MotorPower>("commands/motor_power", rclcpp::QoS(10), std::bind(&KobukiRos::subscribeMotorPower, this, std::placeholders::_1));
  controller_info_command_subscriber_ =  this->create_subscription<kobuki_ros_interfaces::msg::ControllerInfo>("commands/controller_info", rclcpp::QoS(10), std::bind(&KobukiRos::subscribeControllerInfoCommand, this, std::placeholders::_1));

  /*********************
   ** Slots
   **********************/
  slot_stream_data_.connect("/kobuki/stream_data");
  slot_version_info_.connect("/kobuki/version_info");
  slot_controller_info_.connect("/kobuki/controller_info");
  slot_button_event_.connect("/kobuki/button_event");
  slot_bumper_event_.connect("/kobuki/bumper_event");
  slot_cliff_event_.connect("/kobuki/cliff_event");
  slot_wheel_event_.connect("/kobuki/wheel_event");
  slot_power_event_.connect("/kobuki/power_event");
  slot_input_event_.connect("/kobuki/input_event");
  slot_robot_event_.connect("/kobuki/robot_event");
  slot_debug_.connect("/kobuki/debug");
  slot_info_.connect("/kobuki/info");
  slot_warn_.connect("/kobuki/warning");
  slot_error_.connect("/kobuki/error");
  slot_raw_data_command_.connect("/kobuki/raw_data_command");
  slot_raw_data_stream_.connect("/kobuki/raw_data_stream");
  slot_raw_control_command_.connect("/kobuki/raw_control_command");

  /*********************
   ** Driver Parameters
   **********************/
  kobuki::Parameters parameters;

  parameters.log_level = kobuki::LogLevel::NONE; // disable since we are rewiring to ros slots
  parameters.enable_acceleration_limiter = this->declare_parameter("acceleration_limiter", false);
  parameters.battery_capacity = this->declare_parameter("battery_capacity", kobuki::Battery::capacity);
  parameters.battery_low = this->declare_parameter("battery_low", kobuki::Battery::low);
  parameters.battery_dangerous = this->declare_parameter("battery_dangerous", kobuki::Battery::dangerous);
  parameters.sigslots_namespace = "/kobuki";  // configure the first part of the sigslot namespace
  parameters.device_port = this->declare_parameter("device_port", "");

  if (parameters.device_port.empty()) {
    throw std::runtime_error("Kobuki : no device port given on the parameter server (e.g. /dev/ttyUSB0).");
  }

  double cmd_vel_timeout_sec = this->declare_parameter("cmd_vel_timeout_sec", 0.6);
  RCLCPP_INFO(get_logger(), "Kobuki : Velocity commands timeout: %f seconds.", cmd_vel_timeout_sec);

  std::string odom_frame = this->declare_parameter("odom_frame", std::string("odom"));
  RCLCPP_INFO(get_logger(), "Kobuki : using odom_frame [%s].", odom_frame.c_str());

  std::string base_frame = this->declare_parameter("base_frame", std::string("base"));
  RCLCPP_INFO(get_logger(), "Kobuki : using base_frame [%s].", base_frame.c_str());

  bool publish_tf = this->declare_parameter("publish_tf", true);
  if (publish_tf) {
    RCLCPP_INFO(get_logger(), "Kobuki : publishing transforms.");
  }
  else {
    RCLCPP_INFO(get_logger(), "Kobuki : not publishing transforms (see robot_pose_ekf).");
  }

  bool use_imu_heading = this->declare_parameter("use_imu_heading", true);
  if (use_imu_heading) {
    RCLCPP_INFO(get_logger(), "Kobuki : using imu data for heading.");
  }
  else {
    RCLCPP_INFO(get_logger(), "Kobuki : using encoders for heading (see robot_pose_ekf).");
  }

  /*********************
   ** Joint States
   **********************/
  std::string robot_description, wheel_left_joint_name, wheel_right_joint_name;

  wheel_left_joint_name = this->declare_parameter("wheel_left_joint_name", std::string("wheel_left_joint"));
  wheel_right_joint_name = this->declare_parameter("wheel_right_joint_name", std::string("wheel_right_joint"));

  joint_states_.name.push_back(wheel_left_joint_name);
  joint_states_.name.push_back(wheel_right_joint_name);
  joint_states_.position.resize(2, 0.0);
  joint_states_.velocity.resize(2, 0.0);
  joint_states_.effort.resize(2, 0.0);

  /*********************
   ** Validation
   **********************/
  if (!parameters.validate()) {
    RCLCPP_ERROR(get_logger(), "Kobuki : parameter configuration failed");
    RCLCPP_ERROR(get_logger(), "Kobuki : %s", parameters.error_msg.c_str());
    throw std::runtime_error("Failed to validate parameters");
  }
  else {
    if (parameters.simulation) {
      RCLCPP_INFO(get_logger(), "Kobuki : driver going into loopback (simulation) mode.");
    }
    else {
      RCLCPP_INFO(get_logger(), "Kobuki : configured for connection on device_port %s", parameters.device_port.c_str());
      RCLCPP_INFO(get_logger(), "Kobuki : driver running in normal (non-simulation) mode");
    }
  }

  odometry_ = std::make_unique<Odometry>(cmd_vel_timeout_sec, odom_frame, base_frame, publish_tf, use_imu_heading, this->get_clock()->now());

  /*********************
   ** Driver Init
   **********************/
  try {
    kobuki_.init(parameters);
    rclcpp::sleep_for(std::chrono::milliseconds(250));  // wait for some data to come in.
    if (!kobuki_.isAlive()) {
      RCLCPP_WARN(get_logger(), "Kobuki : no data stream, is kobuki turned on?");
      // simply turning kobuki on while spin()'ing should resurrect the situation.
    }
    kobuki_.enable();
  }
  catch (const ecl::StandardException &e) {
    switch (e.flag()) {
      case (ecl::OpenError):
        RCLCPP_ERROR(get_logger(), "Kobuki : could not open connection [%s].", parameters.device_port.c_str());
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Kobuki : initialisation failed");
        RCLCPP_DEBUG(get_logger(), e.what());
        break;
    }
    throw;
  }

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&KobukiRos::update, this));
}

/**
 * This will wait some time while kobuki internally closes its threads and destructs
 * itself.
 */
KobukiRos::~KobukiRos()
{
}

/**
 * This is a worker function that runs in a background thread initiated by
 * the nodelet. It gathers diagnostics information from the kobuki driver,
 * and broadcasts the results to the rest of the ros ecosystem.
 *
 * Note that the actual driver data is collected via the slot callbacks in this class.
 */
void KobukiRos::update()
{
  if (kobuki_.isShutdown()) {
    RCLCPP_ERROR(get_logger(), "Driver has shutdown.");
    return;
  }

  if (kobuki_.isEnabled() && odometry_->commandTimeout(this->get_clock()->now())) {
    if (!cmd_vel_timed_out_) {
      kobuki_.setBaseControl(0, 0);
      cmd_vel_timed_out_ = true;
      RCLCPP_WARN(get_logger(), "Incoming velocity commands not received for more than %.2f seconds -> zero'ing velocity commands", odometry_->timeout().seconds());
    }
  }
  else {
    cmd_vel_timed_out_ = false;
  }

  bool is_alive = kobuki_.isAlive();
  if (watchdog_diagnostics_.isAlive() && !is_alive) {
    if (!serial_timed_out_) {
      RCLCPP_ERROR(get_logger(), "Timed out while waiting for serial data stream.");
      serial_timed_out_ = true;
    }
    else {
      serial_timed_out_ = false;
    }
  }

  watchdog_diagnostics_.update(is_alive);
  battery_diagnostics_.update(kobuki_.batteryStatus());
  cliff_diagnostics_.update(kobuki_.getCoreSensorData().cliff, kobuki_.getCliffData());
  bumper_diagnostics_.update(kobuki_.getCoreSensorData().bumper);
  wheel_diagnostics_.update(kobuki_.getCoreSensorData().wheel_drop);
  motor_diagnostics_.update(kobuki_.getCurrentData().current);
  state_diagnostics_.update(kobuki_.isEnabled());
  gyro_diagnostics_.update(kobuki_.getInertiaData().angle);
  dinput_diagnostics_.update(kobuki_.getGpInputData().digital_input);
  ainput_diagnostics_.update(kobuki_.getGpInputData().analog_input);
  updater_.force_update();
}

void KobukiRos::subscribeVelocityCommand(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
  if (kobuki_.isEnabled()) {
    // For now assuming this is in the robot frame, but probably this
    // should be global frame and require a transform
    //double vx = msg->linear.x;        // in (m/s)
    //double wz = msg->angular.z;       // in (rad/s)
    RCLCPP_DEBUG(get_logger(), "Kobuki : velocity command received [%f],[%f]", msg->linear.x, msg->angular.z);
    kobuki_.setBaseControl(msg->linear.x, msg->angular.z);
    odometry_->resetTimeout(this->get_clock()->now());
  }
}

void KobukiRos::subscribeLed1Command(const std::shared_ptr<kobuki_ros_interfaces::msg::Led> msg)
{
  switch (msg->value) {
    case kobuki_ros_interfaces::msg::Led::GREEN:
      kobuki_.setLed(kobuki::Led1, kobuki::Green);
      break;
    case kobuki_ros_interfaces::msg::Led::ORANGE:
      kobuki_.setLed(kobuki::Led1, kobuki::Orange);
      break;
    case kobuki_ros_interfaces::msg::Led::RED:
      kobuki_.setLed(kobuki::Led1, kobuki::Red);
      break;
    case kobuki_ros_interfaces::msg::Led::BLACK:
      kobuki_.setLed(kobuki::Led1, kobuki::Black);
      break;
    default:
      RCLCPP_WARN(get_logger(), "Kobuki : led 1 command value invalid.");
      break;
  }
}

void KobukiRos::subscribeLed2Command(const std::shared_ptr<kobuki_ros_interfaces::msg::Led> msg)
{
  switch (msg->value) {
    case kobuki_ros_interfaces::msg::Led::GREEN:
      kobuki_.setLed(kobuki::Led2, kobuki::Green);
      break;
    case kobuki_ros_interfaces::msg::Led::ORANGE:
      kobuki_.setLed(kobuki::Led2, kobuki::Orange);
      break;
    case kobuki_ros_interfaces::msg::Led::RED:
      kobuki_.setLed(kobuki::Led2, kobuki::Red);
      break;
    case kobuki_ros_interfaces::msg::Led::BLACK:
      kobuki_.setLed(kobuki::Led2, kobuki::Black);
      break;
    default:
      RCLCPP_WARN(get_logger(), "Kobuki : led 2 command value invalid.");
      break;
  }
}

void KobukiRos::subscribeDigitalOutputCommand(const std::shared_ptr<kobuki_ros_interfaces::msg::DigitalOutput> msg)
{
  kobuki::DigitalOutput digital_output;
  for (unsigned int i = 0; i < 4; ++i) {
    digital_output.values[i] = msg->values[i];
    digital_output.mask[i] = msg->mask[i];
  }
  kobuki_.setDigitalOutput(digital_output);
}

void KobukiRos::subscribeExternalPowerCommand(const std::shared_ptr<kobuki_ros_interfaces::msg::ExternalPower> msg)
{
  // Validate message
  if (!((msg->source == kobuki_ros_interfaces::msg::ExternalPower::PWR_3_3V1A) ||
        (msg->source == kobuki_ros_interfaces::msg::ExternalPower::PWR_5V1A) ||
        (msg->source == kobuki_ros_interfaces::msg::ExternalPower::PWR_12V5A) ||
        (msg->source == kobuki_ros_interfaces::msg::ExternalPower::PWR_12V1_5A))) {
    RCLCPP_ERROR(get_logger(), "Kobuki : Power source %u does not exist!", (unsigned int)msg->source);
    return;
  }
  if (!((msg->state == kobuki_ros_interfaces::msg::ExternalPower::OFF) ||
      (msg->state == kobuki_ros_interfaces::msg::ExternalPower::ON))) {
    RCLCPP_ERROR(get_logger(), "Kobuki : Power source state %u does not exist!", (unsigned int)msg->state);
    return;
  }

  kobuki::DigitalOutput digital_output;
  for (unsigned int i = 0; i < 4; ++i) {
    if (i == msg->source) {
      if (msg->state) {
        digital_output.values[i] = true; // turn source on
        RCLCPP_INFO(get_logger(), "Kobuki : Turning on external power source %u.", (unsigned int)msg->source);
      }
      else {
        digital_output.values[i] = false; // turn source off
        RCLCPP_INFO(get_logger(), "Kobuki : Turning off external power source %u.", (unsigned int)msg->source);
      }
      digital_output.mask[i] = true; // change source state
    }
    else {
      digital_output.values[i] = false; // values doesn't matter here, since mask is set false, what means ignoring
      digital_output.mask[i] = false;
    }
  }
  kobuki_.setExternalPower(digital_output);
}

/**
 * @brief Play a predefined sound (single sound or sound sequence)
 */
void KobukiRos::subscribeSoundCommand(const std::shared_ptr<kobuki_ros_interfaces::msg::Sound> msg)
{
  if (msg->value == kobuki_ros_interfaces::msg::Sound::ON) {
    kobuki_.playSoundSequence(kobuki::On);
  }
  else if (msg->value == kobuki_ros_interfaces::msg::Sound::OFF) {
    kobuki_.playSoundSequence(kobuki::Off);
  }
  else if (msg->value == kobuki_ros_interfaces::msg::Sound::RECHARGE) {
    kobuki_.playSoundSequence(kobuki::Recharge);
  }
  else if (msg->value == kobuki_ros_interfaces::msg::Sound::BUTTON) {
    kobuki_.playSoundSequence(kobuki::Button);
  }
  else if (msg->value == kobuki_ros_interfaces::msg::Sound::ERROR) {
    kobuki_.playSoundSequence(kobuki::Error);
  }
  else if (msg->value == kobuki_ros_interfaces::msg::Sound::CLEANINGSTART) {
    kobuki_.playSoundSequence(kobuki::CleaningStart);
  }
  else if (msg->value == kobuki_ros_interfaces::msg::Sound::CLEANINGEND) {
    kobuki_.playSoundSequence(kobuki::CleaningEnd);
  }
  else {
    RCLCPP_WARN(get_logger(), "Kobuki : Invalid sound command! There is no sound stored for value '%d'.", msg->value);
  }
}

/**
 * @brief Reset the odometry variables.
 */
void KobukiRos::subscribeResetOdometry(const std::shared_ptr<std_msgs::msg::Empty> msg)
{
  (void)msg;
  RCLCPP_INFO(get_logger(), "Kobuki : Resetting the odometry.");
  joint_states_.position[0] = 0.0; // wheel_left
  joint_states_.velocity[0] = 0.0;
  joint_states_.position[1] = 0.0; // wheel_right
  joint_states_.velocity[1] = 0.0;
  odometry_->resetOdometry();
  kobuki_.resetOdometry();
}

void KobukiRos::subscribeMotorPower(const std::shared_ptr<kobuki_ros_interfaces::msg::MotorPower> msg)
{
  if (msg->state == kobuki_ros_interfaces::msg::MotorPower::ON) {
    RCLCPP_INFO(get_logger(), "Kobuki : Firing up the motors.");
    kobuki_.enable();
    odometry_->resetTimeout(this->get_clock()->now());
  }
  else if (msg->state == kobuki_ros_interfaces::msg::MotorPower::OFF) {
    RCLCPP_INFO(get_logger(), "Kobuki : Shutting down the motors.");
    kobuki_.disable();
    odometry_->resetTimeout(this->get_clock()->now());
  }
  else {
    RCLCPP_ERROR(get_logger(), "Kobuki : Motor power command specifies unknown state '%u'.", (unsigned int)msg->state);
  }
}

void KobukiRos::subscribeControllerInfoCommand(const std::shared_ptr<kobuki_ros_interfaces::msg::ControllerInfo> msg)
{
  if (msg->p_gain < 0.0f ||  msg->i_gain < 0.0f ||  msg->d_gain < 0.0f) {
    RCLCPP_ERROR(get_logger(), "Kobuki : All controller gains should be positive.");
    return;
  }
  kobuki_.setControllerGain(msg->type,
                            static_cast<unsigned int>(msg->p_gain*1000.0f),
                            static_cast<unsigned int>(msg->i_gain*1000.0f),
                            static_cast<unsigned int>(msg->d_gain*1000.0f));
}

void KobukiRos::processStreamData()
{
  publishWheelState();
  publishSensorState();
  publishDockIRData();
  publishBatteryState();
  publishInertia();
  publishRawInertia();
}

/*****************************************************************************
** Publish Sensor Stream Workers
*****************************************************************************/

void KobukiRos::publishSensorState()
{
  kobuki_ros_interfaces::msg::SensorState state;
  kobuki::CoreSensors::Data data = kobuki_.getCoreSensorData();
  state.header.stamp = this->get_clock()->now();
  state.time_stamp = data.time_stamp; // firmware time stamp
  state.bumper = data.bumper;
  state.wheel_drop = data.wheel_drop;
  state.cliff = data.cliff;
  state.left_encoder = data.left_encoder;
  state.right_encoder = data.right_encoder;
  state.left_pwm = data.left_pwm;
  state.right_pwm = data.right_pwm;
  state.buttons = data.buttons;
  state.charger = data.charger;
  state.battery = data.battery;
  state.over_current = data.over_current;

  kobuki::Cliff::Data cliff_data = kobuki_.getCliffData();
  state.bottom = cliff_data.bottom;

  kobuki::Current::Data current_data = kobuki_.getCurrentData();
  state.current = current_data.current;

  kobuki::GpInput::Data gp_input_data = kobuki_.getGpInputData();
  state.digital_input = gp_input_data.digital_input;
  for (unsigned int i = 0; i < gp_input_data.analog_input.size(); ++i) {
    state.analog_input.push_back(gp_input_data.analog_input[i]);
  }

  sensor_state_publisher_->publish(state);
}

void KobukiRos::publishWheelState()
{
  // Take latest encoders and gyro data
  ecl::linear_algebra::Vector3d pose_update, pose_update_rates;
  kobuki_.updateOdometry(pose_update, pose_update_rates);
  kobuki_.getWheelJointStates(joint_states_.position[0], joint_states_.velocity[0],   // left wheel
                              joint_states_.position[1], joint_states_.velocity[1]);  // right wheel

  // Update and publish odometry and joint states
  odometry_->update(pose_update, pose_update_rates, kobuki_.getHeading(), kobuki_.getAngularVelocity(), this->get_clock()->now());
  std::unique_ptr<geometry_msgs::msg::TransformStamped> odom_update = odometry_->getTransform();
  if (odom_update != nullptr) {
    odom_broadcaster_->sendTransform(*odom_update);
  }
  odom_publisher_->publish(std::move(odometry_->getOdometry()));

  joint_states_.header.stamp = this->get_clock()->now();
  joint_state_publisher_->publish(joint_states_);
}

void KobukiRos::publishInertia()
{
  // Publish as unique pointer to leverage the zero-copy pub/sub feature
  auto msg = std::make_unique<sensor_msgs::msg::Imu>();

  msg->header.frame_id = "gyro_link";
  msg->header.stamp = this->get_clock()->now();

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, kobuki_.getHeading());
  msg->orientation.x = q.x();
  msg->orientation.y = q.y();
  msg->orientation.z = q.z();
  msg->orientation.w = q.w();

  // set a non-zero covariance on unused dimensions (pitch and roll); this is a requirement of robot_pose_ekf
  // set yaw covariance as very low, to make it dominate over the odometry heading when combined
  // 1: fill once, as its always the same;  2: using an invented value; cannot we get a realistic estimation?
  msg->orientation_covariance[0] = DBL_MAX;
  msg->orientation_covariance[4] = DBL_MAX;
  msg->orientation_covariance[8] = 0.05;

  // fill angular velocity; we ignore acceleration for now
  msg->angular_velocity.z = kobuki_.getAngularVelocity();

  // angular velocity covariance; useless by now, but robot_pose_ekf's
  // roadmap claims that it will compute velocities in the future
  msg->angular_velocity_covariance[0] = DBL_MAX;
  msg->angular_velocity_covariance[4] = DBL_MAX;
  msg->angular_velocity_covariance[8] = 0.05;

  imu_data_publisher_->publish(std::move(msg));
}

void KobukiRos::publishRawInertia()
{
    // Publish as unique pointer to leverage the zero-copy pub/sub feature
  kobuki::ThreeAxisGyro::Data data = kobuki_.getRawInertiaData();

  rclcpp::Time now = this->get_clock()->now();
  rclcpp::Duration interval(rclcpp::Duration::from_seconds(0.01)); // Time interval between each sensor reading.
  const double digit_to_dps = 0.00875; // digit to deg/s ratio, comes from datasheet of 3d gyro[L3G4200D].
  unsigned int length = data.followed_data_length / 3;
  for (unsigned int i = 0; i < length; i++) {
    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    // Each sensor reading has id, that circulate 0 to 255.
    msg->header.frame_id = "gyro_link";

    // Update rate of 3d gyro sensor is 100 Hz, but robot's update rate is 50 Hz.
    // So, here is some compensation.
    // See also https://github.com/yujinrobot/kobuki/issues/216
    msg->header.stamp = now - interval * (length-i-1);

    // Sensing axis of 3d gyro is not match with robot. It is rotated 90 degree counterclockwise about z-axis.
    msg->angular_velocity.x = angles::from_degrees( -digit_to_dps * (short)data.data[i*3+1] );
    msg->angular_velocity.y = angles::from_degrees(  digit_to_dps * (short)data.data[i*3+0] );
    msg->angular_velocity.z = angles::from_degrees(  digit_to_dps * (short)data.data[i*3+2] );

    raw_imu_data_publisher_->publish(std::move(msg));
  }
}

void KobukiRos::publishDockIRData()
{
  kobuki::DockIR::Data data = kobuki_.getDockIRData();

  // Publish as unique pointer to leverage the zero-copy pub/sub feature
  auto msg = std::make_unique<kobuki_ros_interfaces::msg::DockInfraRed>();

  msg->header.frame_id = "dock_ir_link";
  msg->header.stamp = this->get_clock()->now();

  msg->data.push_back(data.docking[0]);
  msg->data.push_back(data.docking[1]);
  msg->data.push_back(data.docking[2]);

  dock_ir_publisher_->publish(std::move(msg));
}

void KobukiRos::publishBatteryState()
{
  // lazy publisher
  if (battery_state_publisher_->get_subscription_count() == 0 &&
      battery_state_publisher_->get_intra_process_subscription_count() == 0)
  {
    return;
  }

  auto msg = std::make_unique<sensor_msgs::msg::BatteryState>();
  auto battery = kobuki_.batteryStatus();

  msg->header.frame_id = "base_link";
  msg->header.stamp = get_clock()->now();
  msg->voltage = battery.voltage;
  msg->temperature = std::numeric_limits<float>::quiet_NaN();
  msg->current = std::numeric_limits<float>::quiet_NaN();
  msg->charge = std::numeric_limits<float>::quiet_NaN();
  msg->capacity = std::numeric_limits<float>::quiet_NaN();
  msg->design_capacity = std::numeric_limits<float>::quiet_NaN();
  switch (battery.charging_state) {
    case kobuki::Battery::Discharging:
      msg->power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
      break;
    case kobuki::Battery::Charged:
      msg->power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
      break;
    case kobuki::Battery::Charging:
      msg->power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
      break;
  }
  msg->power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  msg->power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  msg->location = "";
  msg->serial_number = "";
  msg->percentage = battery.percent();
  msg->present = true;
  battery_state_publisher_->publish(std::move(msg));
}

/*****************************************************************************
** Non Default Stream Packets
*****************************************************************************/
/**
 * @brief Publish fw, hw, sw version information.
 *
 * The driver will only gather this data when initialising so it is
 * important that this publisher is latched.
 */
void KobukiRos::publishVersionInfo(const kobuki::VersionInfo &version_info)
{
  auto msg = std::make_unique<kobuki_ros_interfaces::msg::VersionInfo>();

  msg->firmware = kobuki::VersionInfo::toString(version_info.firmware);
  msg->hardware = kobuki::VersionInfo::toString(version_info.hardware);
  msg->software = kobuki::VersionInfo::getSoftwareVersion();

  msg->udid.resize(3);
  msg->udid[0] = version_info.udid0;
  msg->udid[1] = version_info.udid1;
  msg->udid[2] = version_info.udid2;

  // Set available features mask depending on firmware and driver versions
  if (version_info.firmware > 65536) {  // 1.0.0
    msg->features |= kobuki_ros_interfaces::msg::VersionInfo::SMOOTH_MOVE_START;
    msg->features |= kobuki_ros_interfaces::msg::VersionInfo::GYROSCOPE_3D_DATA;
  }
  if (version_info.firmware > 65792) {  // 1.1.0
    // msg->features |= kobuki_ros_interfaces::msg::VersionInfo::SOMETHING_JINCHA_FANCY;
  }
  // if (msg->firmware > ...

  version_info_publisher_->publish(std::move(msg));
}

void KobukiRos::publishControllerInfo()
{
  auto msg = std::make_unique<kobuki_ros_interfaces::msg::ControllerInfo>();
  kobuki::ControllerInfo::Data data = kobuki_.getControllerInfoData();

  msg->type = data.type;
  msg->p_gain = static_cast<float>(data.p_gain) * 0.001f;
  msg->i_gain = static_cast<float>(data.i_gain) * 0.001f;
  msg->d_gain = static_cast<float>(data.d_gain) * 0.001f;

  controller_info_publisher_->publish(std::move(msg));
}

/*****************************************************************************
** Events
*****************************************************************************/

void KobukiRos::publishButtonEvent(const kobuki::ButtonEvent &event)
{
  auto msg = std::make_unique<kobuki_ros_interfaces::msg::ButtonEvent>();
  switch(event.state) {
    case kobuki::ButtonEvent::Pressed:
      msg->state = kobuki_ros_interfaces::msg::ButtonEvent::PRESSED;
      break;
    case kobuki::ButtonEvent::Released:
      msg->state = kobuki_ros_interfaces::msg::ButtonEvent::RELEASED;
      break;
    default:
      break;
  }
  switch(event.button) {
    case kobuki::ButtonEvent::Button0:
      msg->button = kobuki_ros_interfaces::msg::ButtonEvent::BUTTON0;
      break;
    case kobuki::ButtonEvent::Button1:
      msg->button = kobuki_ros_interfaces::msg::ButtonEvent::BUTTON1;
      break;
    case kobuki::ButtonEvent::Button2:
      msg->button = kobuki_ros_interfaces::msg::ButtonEvent::BUTTON2;
      break;
    default:
      break;
  }
  button_event_publisher_->publish(std::move(msg));
}

void KobukiRos::publishBumperEvent(const kobuki::BumperEvent &event)
{
  auto msg = std::make_unique<kobuki_ros_interfaces::msg::BumperEvent>();
  switch(event.state) {
    case kobuki::BumperEvent::Pressed:
      msg->state = kobuki_ros_interfaces::msg::BumperEvent::PRESSED;
      break;
    case kobuki::BumperEvent::Released:
      msg->state = kobuki_ros_interfaces::msg::BumperEvent::RELEASED;
      break;
    default:
      break;
  }
  switch(event.bumper) {
    case kobuki::BumperEvent::Left:
      msg->bumper = kobuki_ros_interfaces::msg::BumperEvent::LEFT;
      break;
    case kobuki::BumperEvent::Center:
      msg->bumper = kobuki_ros_interfaces::msg::BumperEvent::CENTER;
      break;
    case kobuki::BumperEvent::Right:
      msg->bumper = kobuki_ros_interfaces::msg::BumperEvent::RIGHT;
      break;
    default:
      break;
  }
  bumper_event_publisher_->publish(std::move(msg));
}

void KobukiRos::publishCliffEvent(const kobuki::CliffEvent &event)
{
  auto msg = std::make_unique<kobuki_ros_interfaces::msg::CliffEvent>();
  switch(event.state) {
    case kobuki::CliffEvent::Floor:
      msg->state = kobuki_ros_interfaces::msg::CliffEvent::FLOOR;
      break;
    case kobuki::CliffEvent::Cliff:
      msg->state = kobuki_ros_interfaces::msg::CliffEvent::CLIFF;
      break;
    default:
      break;
  }
  switch(event.sensor) {
    case kobuki::CliffEvent::Left:
      msg->sensor = kobuki_ros_interfaces::msg::CliffEvent::LEFT;
      break;
    case kobuki::CliffEvent::Center:
      msg->sensor = kobuki_ros_interfaces::msg::CliffEvent::CENTER;
      break;
    case kobuki::CliffEvent::Right:
      msg->sensor = kobuki_ros_interfaces::msg::CliffEvent::RIGHT;
      break;
    default:
      break;
  }
  msg->bottom = event.bottom;
  cliff_event_publisher_->publish(std::move(msg));
}

void KobukiRos::publishWheelEvent(const kobuki::WheelEvent &event)
{
  auto msg = std::make_unique<kobuki_ros_interfaces::msg::WheelDropEvent>();
  switch(event.state) {
    case kobuki::WheelEvent::Dropped:
      msg->state = kobuki_ros_interfaces::msg::WheelDropEvent::DROPPED;
      break;
    case kobuki::WheelEvent::Raised:
      msg->state = kobuki_ros_interfaces::msg::WheelDropEvent::RAISED;
      break;
    default:
      break;
  }
  switch(event.wheel) {
    case kobuki::WheelEvent::Left:
      msg->wheel = kobuki_ros_interfaces::msg::WheelDropEvent::LEFT;
      break;
    case kobuki::WheelEvent::Right:
      msg->wheel = kobuki_ros_interfaces::msg::WheelDropEvent::RIGHT;
      break;
    default:
      break;
  }
  wheel_event_publisher_->publish(std::move(msg));
}

void KobukiRos::publishPowerEvent(const kobuki::PowerEvent &event)
{
  auto msg = std::make_unique<kobuki_ros_interfaces::msg::PowerSystemEvent>();
  switch(event.event) {
    case kobuki::PowerEvent::Unplugged:
      msg->event = kobuki_ros_interfaces::msg::PowerSystemEvent::UNPLUGGED;
      break;
    case kobuki::PowerEvent::PluggedToAdapter:
      msg->event = kobuki_ros_interfaces::msg::PowerSystemEvent::PLUGGED_TO_ADAPTER;
      break;
    case kobuki::PowerEvent::PluggedToDockbase:
      msg->event = kobuki_ros_interfaces::msg::PowerSystemEvent::PLUGGED_TO_DOCKBASE;
      break;
    case kobuki::PowerEvent::ChargeCompleted:
      msg->event = kobuki_ros_interfaces::msg::PowerSystemEvent::CHARGE_COMPLETED;
      break;
    case kobuki::PowerEvent::BatteryLow:
      msg->event = kobuki_ros_interfaces::msg::PowerSystemEvent::BATTERY_LOW;
      break;
    case kobuki::PowerEvent::BatteryCritical:
      msg->event = kobuki_ros_interfaces::msg::PowerSystemEvent::BATTERY_CRITICAL;
      break;
    default:
      break;
  }
  power_event_publisher_->publish(std::move(msg));
}

void KobukiRos::publishInputEvent(const kobuki::InputEvent &event)
{
  auto msg = std::make_unique<kobuki_ros_interfaces::msg::DigitalInputEvent>();
  for (unsigned int i = 0; i < msg->values.size(); i++) {
    msg->values[i] = event.values[i];
  }
  input_event_publisher_->publish(std::move(msg));
}

void KobukiRos::publishRobotEvent(const kobuki::RobotEvent &event)
{
  auto msg = std::make_unique<kobuki_ros_interfaces::msg::RobotStateEvent>();
  switch(event.state) {
    case kobuki::RobotEvent::Online:
      msg->state = kobuki_ros_interfaces::msg::RobotStateEvent::ONLINE;
      break;
    case kobuki::RobotEvent::Offline:
      msg->state = kobuki_ros_interfaces::msg::RobotStateEvent::OFFLINE;
      break;
    default:
      break;
  }

  robot_event_publisher_->publish(std::move(msg));
}

/**
 * @brief Prints the raw data stream to a publisher.
 *
 * This is a lazy publisher, it only publishes if someone is listening. It publishes the
 * hex byte values of the raw data commands. Useful for debugging command to protocol
 * byte packets to the firmware.
 *
 * The signal which calls this
 * function is sending a copy of the buffer (don't worry about mutexes). Be ideal if we used
 * const PacketFinder::BufferType here, but haven't updated PushPop to work with consts yet.
 *
 * @param buffer
 */
void KobukiRos::publishRawDataCommand(kobuki::Command::Buffer &buffer)
{
  if (raw_data_command_publisher_->get_subscription_count() == 0 &&
    raw_data_command_publisher_->get_intra_process_subscription_count() == 0)   // no one listening?
  {
    return;                                     // avoid publishing
  }

  std::ostringstream ostream;
  kobuki::Command::Buffer::Formatter format;
  ostream << format(buffer); // convert to an easily readable hex string.
  auto s = std::make_unique<std_msgs::msg::String>();
  s->data = ostream.str();
  raw_data_command_publisher_->publish(std::move(s));
}

/**
 * @brief Prints the raw data stream to a publisher.
 *
 * This is a lazy publisher, it only publishes if someone is listening. It publishes the
 * hex byte values of the raw data (incoming) stream. Useful for checking when bytes get
 * mangled.
 *
 * The signal which calls this
 * function is sending a copy of the buffer (don't worry about mutexes). Be ideal if we used
 * const PacketFinder::BufferType here, but haven't updated PushPop to work with consts yet.
 *
 * @param buffer
 */
void KobukiRos::publishRawDataStream(kobuki::PacketFinder::BufferType &buffer)
{
  if (raw_data_stream_publisher_->get_subscription_count() == 0 &&
    raw_data_stream_publisher_->get_intra_process_subscription_count() == 0)   // no one listening?
  {
    return;                                     // avoid publishing
  }

  std::ostringstream ostream;
  ostream << "{ " ;
  ostream << std::setfill('0') << std::uppercase;
  for (unsigned int i = 0; i < buffer.size(); i++) {
    ostream << std::hex << std::setw(2) << static_cast<unsigned int>(buffer[i]) << " " << std::dec;
  }
  ostream << "}";
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = ostream.str();
  raw_data_stream_publisher_->publish(std::move(msg));
}

void KobukiRos::publishRawControlCommand(const std::vector<short> &velocity_commands)
{
  auto msg = std::make_unique<std_msgs::msg::Int16MultiArray>();
  msg->data = velocity_commands;
  raw_control_command_publisher_->publish(std::move(msg));
}

} // namespace kobuki_node

RCLCPP_COMPONENTS_REGISTER_NODE(kobuki_node::KobukiRos)
