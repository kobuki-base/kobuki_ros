/**
 * @file /kobuki_random_walker/kobuki_random_walker.hpp
 *
 * @brief A controller implementing a simple random walker algorithm
 *
 * License: BSD
 *   https://raw.github.com/yujinrobot/kobuki/hydro-devel/kobuki_random_walker/LICENSE
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef RANDOM_WALKER_HPP_
#define RANDOM_WALKER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <random>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include <kobuki_ros_interfaces/msg/bumper_event.hpp>
#include <kobuki_ros_interfaces/msg/cliff_event.hpp>
#include <kobuki_ros_interfaces/msg/led.hpp>
#include <kobuki_ros_interfaces/msg/wheel_drop_event.hpp>

namespace kobuki_random_walker
{

/**
 * @ brief A controller implementing a simple random walker algorithm
 *
 * Controller moves the robot around, changing direction whenever a bumper or cliff event occurs
 * For changing direction random angles are used.
 */
class RandomWalkerNode : public rclcpp::Node
{
public:
  explicit RandomWalkerNode(const rclcpp::NodeOptions & options);
  ~RandomWalkerNode(){};

private:
  /// Subscribers
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr enable_controller_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr disable_controller_subscriber_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_event_subscriber_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::CliffEvent>::SharedPtr cliff_event_subscriber_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::WheelDropEvent>::SharedPtr wheel_drop_event_subscriber_;
  /// Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led1_publisher_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led2_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::random_device random_dev_;
  std::mt19937 mt_;
  std::uniform_real_distribution<double> rand_distribution_;

  /// Flag for whether controller is active
  bool controller_active_;
  /// Flag for changing direction
  bool change_direction_;
  /// Flag for stopping
  bool stop_;
  /// Flag for left bumper's state
  bool bumper_left_pressed_;
  /// Flag for center bumper's state
  bool bumper_center_pressed_;
  /// Flag for right bumper's state
  bool bumper_right_pressed_;
  /// Flag for left cliff sensor's state
  bool cliff_left_detected_;
  /// Flag for center cliff sensor's state
  bool cliff_center_detected_;
  /// Flag for right cliff sensor's state
  bool cliff_right_detected_;
  /// Flag for left wheel drop sensor's state
  bool wheel_drop_left_detected_;
  /// Flag for right wheel drop sensor's state
  bool wheel_drop_right_detected_;
  /// Flag for bumper LED's state
  bool led_bumper_on_;
  /// Flag for cliff sensor LED's state
  bool led_cliff_on_;
  /// Flag for wheel drop sensor LED's state
  bool led_wheel_drop_on_;
  /// Randomly chosen turning duration
  rclcpp::Duration turning_duration_;
  /// Randomly chosen turning direction
  int turning_direction_;
  /// Start time of turning
  rclcpp::Time turning_start_;
  /// Flag for turning state
  bool turning_;

  bool enable();

  bool disable();

  bool getState();

  void enableCB(const std_msgs::msg::Empty::SharedPtr msg);

  void disableCB(const std_msgs::msg::Empty::SharedPtr msg);

  void bumperEventCB(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg);

  void cliffEventCB(const kobuki_ros_interfaces::msg::CliffEvent::SharedPtr msg);

  void wheelDropEventCB(const kobuki_ros_interfaces::msg::WheelDropEvent::SharedPtr msg);

  void spin();
};

}  // namespace kobuki_random_walker

#endif /* RANDOM_WALKER_HPP_ */
