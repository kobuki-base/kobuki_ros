/**
 * @file /kobuki_random_walker/kobuki_random_walker.cpp
 *
 * @brief A controller implementing a simple random walker algorithm
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/hydro-devel/kobuki_random_walker/LICENSE
 **/

#include <cmath>
#include <functional>
#include <memory>
#include <random>
#include <utility>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/empty.hpp>

#include <kobuki_ros_interfaces/msg/bumper_event.hpp>
#include <kobuki_ros_interfaces/msg/cliff_event.hpp>
#include <kobuki_ros_interfaces/msg/led.hpp>
#include <kobuki_ros_interfaces/msg/wheel_drop_event.hpp>

#include "kobuki_random_walker/random_walker.hpp"

namespace kobuki_random_walker
{

RandomWalkerNode::RandomWalkerNode(const rclcpp::NodeOptions & options) : rclcpp::Node("kobuki_random_walker_node", options),
                                                                          random_dev_(),
                                                                          mt_(random_dev_()),
                                                                          rand_distribution_(-1.0, 1.0),
                                                                          controller_active_(false),
                                                                          change_direction_(false),
                                                                          stop_(false),
                                                                          bumper_left_pressed_(false),
                                                                          bumper_center_pressed_(false),
                                                                          bumper_right_pressed_(false),
                                                                          cliff_left_detected_(false),
                                                                          cliff_center_detected_(false),
                                                                          cliff_right_detected_(false),
                                                                          wheel_drop_left_detected_(false),
                                                                          wheel_drop_right_detected_(false),
                                                                          led_bumper_on_(false),
                                                                          led_cliff_on_(false),
                                                                          led_wheel_drop_on_(false),
                                                                          turning_duration_(rclcpp::Duration::from_seconds(0.0)),
                                                                          turning_direction_(1),
                                                                          turning_(false)
{
  enable_controller_subscriber_ = this->create_subscription<std_msgs::msg::Empty>("enable", rclcpp::QoS(10), std::bind(&RandomWalkerNode::enableCB, this, std::placeholders::_1));
  disable_controller_subscriber_ = create_subscription<std_msgs::msg::Empty>("disable", rclcpp::QoS(10), std::bind(&RandomWalkerNode::disableCB, this, std::placeholders::_1));
  bumper_event_subscriber_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>("events/bumper", rclcpp::QoS(10), std::bind(&RandomWalkerNode::bumperEventCB, this, std::placeholders::_1));
  cliff_event_subscriber_ = create_subscription<kobuki_ros_interfaces::msg::CliffEvent>("events/cliff", rclcpp::QoS(10), std::bind(&RandomWalkerNode::cliffEventCB, this, std::placeholders::_1));
  wheel_drop_event_subscriber_ = create_subscription<kobuki_ros_interfaces::msg::WheelDropEvent>("events/wheel_drop", rclcpp::QoS(10), std::bind(&RandomWalkerNode::wheelDropEventCB, this, std::placeholders::_1));

  cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("commands/velocity", 10);
  led1_publisher_ = create_publisher<kobuki_ros_interfaces::msg::Led>("commands/led1", 10);
  led2_publisher_ = create_publisher<kobuki_ros_interfaces::msg::Led>("commands/led2", 10);

  double linear_velocity = declare_parameter("linear_velocity", 0.5);
  double angular_velocity = declare_parameter("angular_velocity", 0.1);

  RCLCPP_INFO(get_logger(), "Velocity parameters: linear velocity = %f, angular_velocity = %f", linear_velocity, angular_velocity);

  this->enable(); // enable controller

  timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&RandomWalkerNode::spin, this));
}

bool RandomWalkerNode::enable()
{
  if (controller_active_)
  {
    return false;
  }

  controller_active_ = true;
  return true;
}

bool RandomWalkerNode::disable()
{
  if (!controller_active_)
  {
    return false;
  }

  controller_active_ = false;
  return true;
}

bool RandomWalkerNode::getState()
{
  return controller_active_;
}

void RandomWalkerNode::enableCB(const std_msgs::msg::Empty::SharedPtr msg)
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

void RandomWalkerNode::disableCB(const std_msgs::msg::Empty::SharedPtr msg)
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

void RandomWalkerNode::bumperEventCB(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg)
{
  if (this->getState()) // check, if the controller is active
  {
    if (msg->state == kobuki_ros_interfaces::msg::BumperEvent::PRESSED)
    {
      switch (msg->bumper)
      {
        case kobuki_ros_interfaces::msg::BumperEvent::LEFT:
          if (!bumper_left_pressed_)
          {
            bumper_left_pressed_ = true;
            change_direction_ = true;
          }
          break;
        case kobuki_ros_interfaces::msg::BumperEvent::CENTER:
          if (!bumper_center_pressed_)
          {
            bumper_center_pressed_ = true;
            change_direction_ = true;
          }
          break;
        case kobuki_ros_interfaces::msg::BumperEvent::RIGHT:
          if (!bumper_right_pressed_)
          {
            bumper_right_pressed_ = true;
            change_direction_ = true;
          }
          break;
      }
    }
    else // kobuki_ros_interfaces::msg::BumperEvent::RELEASED
    {
      switch (msg->bumper)
      {
        case kobuki_ros_interfaces::msg::BumperEvent::LEFT:    bumper_left_pressed_   = false; break;
        case kobuki_ros_interfaces::msg::BumperEvent::CENTER:  bumper_center_pressed_ = false; break;
        case kobuki_ros_interfaces::msg::BumperEvent::RIGHT:   bumper_right_pressed_  = false; break;
      }
    }
    if (!led_bumper_on_ && (bumper_left_pressed_ || bumper_center_pressed_ || bumper_right_pressed_))
    {
      auto led_msg = std::make_unique<kobuki_ros_interfaces::msg::Led>();
      led_msg->value = kobuki_ros_interfaces::msg::Led::ORANGE;
      led1_publisher_->publish(std::move(led_msg));
      led_bumper_on_ = true;
    }
    else if (led_bumper_on_ && (!bumper_left_pressed_ && !bumper_center_pressed_ && !bumper_right_pressed_))
    {
      auto led_msg = std::make_unique<kobuki_ros_interfaces::msg::Led>();
      led_msg->value = kobuki_ros_interfaces::msg::Led::BLACK;
      led1_publisher_->publish(std::move(led_msg));
      led_bumper_on_ = false;
    }
    if (change_direction_)
    {
      RCLCPP_INFO(get_logger(), "Bumper pressed. Changing direction.");
    }
  }
}

void RandomWalkerNode::cliffEventCB(const kobuki_ros_interfaces::msg::CliffEvent::SharedPtr msg)
{
  if (msg->state == kobuki_ros_interfaces::msg::CliffEvent::CLIFF)
  {
    switch (msg->sensor)
    {
      case kobuki_ros_interfaces::msg::CliffEvent::LEFT:
        if (!cliff_left_detected_)
        {
          cliff_left_detected_ = true;
          change_direction_ = true;
        }
        break;
      case kobuki_ros_interfaces::msg::CliffEvent::CENTER:
        if (!cliff_center_detected_)
        {
          cliff_center_detected_ = true;
          change_direction_ = true;
        }
        break;
      case kobuki_ros_interfaces::msg::CliffEvent::RIGHT:
        if (!cliff_right_detected_)
        {
          cliff_right_detected_ = true;
          change_direction_ = true;
        }
        break;
    }
  }
  else // kobuki_ros_interfaces::BumperEvent::FLOOR
  {
    switch (msg->sensor)
    {
      case kobuki_ros_interfaces::msg::CliffEvent::LEFT:    cliff_left_detected_   = false; break;
      case kobuki_ros_interfaces::msg::CliffEvent::CENTER:  cliff_center_detected_ = false; break;
      case kobuki_ros_interfaces::msg::CliffEvent::RIGHT:   cliff_right_detected_  = false; break;
    }
  }
  if (!led_cliff_on_ && (cliff_left_detected_ || cliff_center_detected_ || cliff_right_detected_))
  {
    auto led_msg = std::make_unique<kobuki_ros_interfaces::msg::Led>();
    led_msg->value = kobuki_ros_interfaces::msg::Led::ORANGE;
    led2_publisher_->publish(std::move(led_msg));
    led_cliff_on_ = true;
  }
  else if (led_cliff_on_ && (!cliff_left_detected_ && !cliff_center_detected_ && !cliff_right_detected_))
  {
    auto led_msg = std::make_unique<kobuki_ros_interfaces::msg::Led>();
    led_msg->value = kobuki_ros_interfaces::msg::Led::BLACK;
    led2_publisher_->publish(std::move(led_msg));
    led_cliff_on_ = false;
  }
  if (change_direction_)
  {
    RCLCPP_INFO(get_logger(), "Cliff detected. Changing direction.");
  }
}

void RandomWalkerNode::wheelDropEventCB(const kobuki_ros_interfaces::msg::WheelDropEvent::SharedPtr msg)
{
  if (msg->state == kobuki_ros_interfaces::msg::WheelDropEvent::DROPPED)
  {
    switch (msg->wheel)
    {
      case kobuki_ros_interfaces::msg::WheelDropEvent::LEFT:
        if (!wheel_drop_left_detected_)
        {
          wheel_drop_left_detected_ = true;
        }
        break;
      case kobuki_ros_interfaces::msg::WheelDropEvent::RIGHT:
        if (!wheel_drop_right_detected_)
        {
          wheel_drop_right_detected_ = true;
        }
        break;
    }
  }
  else // kobuki_ros_interfaces::WheelDropEvent::RAISED
  {
    switch (msg->wheel)
    {
      case kobuki_ros_interfaces::msg::WheelDropEvent::LEFT:    wheel_drop_left_detected_   = false; break;
      case kobuki_ros_interfaces::msg::WheelDropEvent::RIGHT:   wheel_drop_right_detected_  = false; break;
    }
  }
  if (!led_wheel_drop_on_ && (wheel_drop_left_detected_ || wheel_drop_right_detected_))
  {
    kobuki_ros_interfaces::msg::Led led_msg;
    led_msg.value = kobuki_ros_interfaces::msg::Led::RED;
    led1_publisher_->publish(led_msg);
    led2_publisher_->publish(led_msg);
    stop_ = true;
    led_wheel_drop_on_ = true;
  }
  else if (led_wheel_drop_on_ && (!wheel_drop_left_detected_ && !wheel_drop_right_detected_))
  {
    kobuki_ros_interfaces::msg::Led led_msg;
    led_msg.value = kobuki_ros_interfaces::msg::Led::BLACK;
    led1_publisher_->publish(led_msg);
    led2_publisher_->publish(led_msg);
    stop_ = false;
    led_wheel_drop_on_ = false;
  }
  if (change_direction_)
  {
    RCLCPP_INFO(get_logger(), "Wheel(s) dropped. Stopping.");
  }
}

void RandomWalkerNode::spin()
{
  if (this->getState()) // check, if the controller is active
  {
    // Velocity commands
    auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();

    if (stop_)
    {
      cmd_vel_publisher_->publish(std::move(cmd_vel_msg)); // will be all zero when initialised
      return;
    }

    double vel_ang = get_parameter("angular_velocity").get_value<double>();

    if (change_direction_)
    {
      change_direction_ = false;
      // calculate a random turning angle (-180 ... +180) based on the set angular velocity
      // time for turning 180 degrees in seconds = M_PI / angular velocity
      double random_number = rand_distribution_(mt_);
      turning_duration_ = rclcpp::Duration::from_seconds(std::fabs(random_number) * (M_PI / vel_ang));
      // randomly chosen turning direction
      if (random_number >= 0.0)
      {
        turning_direction_ = 1;
      }
      else
      {
        turning_direction_ = -1;
      }
      turning_start_ = now();
      turning_ = true;
      RCLCPP_INFO(get_logger(), "Will rotate %f degrees.", turning_direction_ * turning_duration_.seconds() * vel_ang / M_PI * 180.0);
    }

    if (turning_)
    {
      if ((now() - turning_start_) < turning_duration_)
      {
        cmd_vel_msg->angular.z = turning_direction_ * vel_ang;
        cmd_vel_publisher_->publish(std::move(cmd_vel_msg));
      }
      else
      {
        turning_ = false;
      }
    }
    else
    {
      cmd_vel_msg->linear.x = get_parameter("linear_velocity").get_value<double>();
      cmd_vel_publisher_->publish(std::move(cmd_vel_msg));
    }
  }
}

}  // namespace kobuki_random_walker

RCLCPP_COMPONENTS_REGISTER_NODE(kobuki_random_walker::RandomWalkerNode)
