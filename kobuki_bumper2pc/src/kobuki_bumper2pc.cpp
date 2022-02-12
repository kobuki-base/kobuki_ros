/**
 * @file /src/kobuki_bumper2pc.cpp
 *
 * @brief Bumper to pointcloud nodelet class implementation.
 *
 * @author Jorge Santos, Yujin Robot
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/parameter_events_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "kobuki_ros_interfaces/msg/sensor_state.hpp"

#include "kobuki_bumper2pc/kobuki_bumper2pc.hpp"

namespace kobuki_bumper2pc
{

void Bumper2PcNode::coreSensorCB(const std::shared_ptr<kobuki_ros_interfaces::msg::SensorState> msg)
{
  if (pointcloud_pub_->get_subscription_count() == 0 &&
    pointcloud_pub_->get_intra_process_subscription_count() == 0)   // no one listening?
  {
    return;
  }

  // We publish just one "no events" pc (with all three points far away) and stop spamming when bumper/cliff conditions disappear
  if (! msg->bumper && ! msg->cliff && ! prev_bumper_ && ! prev_cliff_)
  {
    return;
  }

  prev_bumper_ = msg->bumper;
  prev_cliff_  = msg->cliff;

  // We replicate the sensors order of bumper/cliff event messages: LEFT = 0, CENTER = 1 and RIGHT = 2
  // For any of {left/center/right} with no bumper/cliff event, we publish a faraway point that won't get used
  if ((msg->bumper & kobuki_ros_interfaces::msg::SensorState::BUMPER_LEFT) ||
      (msg->cliff  & kobuki_ros_interfaces::msg::SensorState::CLIFF_LEFT))
  {
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &p_side_y_, sizeof(float));
  }
  else
  {
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[1].offset], &P_INF_Y, sizeof(float));
  }

  if ((msg->bumper & kobuki_ros_interfaces::msg::SensorState::BUMPER_CENTRE) ||
      (msg->cliff  & kobuki_ros_interfaces::msg::SensorState::CLIFF_CENTRE))
  {
    float radius = static_cast<float>(get_parameter("pointcloud_radius").get_value<double>());
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &radius, sizeof(float));
  }
  else
  {
    memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
  }

  if ((msg->bumper & kobuki_ros_interfaces::msg::SensorState::BUMPER_RIGHT) ||
      (msg->cliff  & kobuki_ros_interfaces::msg::SensorState::CLIFF_RIGHT))
  {
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &p_side_x_, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &n_side_y_, sizeof(float));
  }
  else
  {
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[0].offset], &P_INF_X, sizeof(float));
    memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[1].offset], &N_INF_Y, sizeof(float));
  }

  pointcloud_.header.stamp = msg->header.stamp;
  pointcloud_pub_->publish(pointcloud_);
}

void Bumper2PcNode::onParameterEvent(
  std::shared_ptr<rcl_interfaces::msg::ParameterEvent> event)
{
  // Filter out events from other nodes
  if (event->node != this->get_fully_qualified_name()) {
    return;
  }

  // Filter for "pointcloud_radius", "pointcloud_height", "side_point_angle", or "base_link_frame" being changed.
  rclcpp::ParameterEventsFilter filter(event, {"pointcloud_radius", "pointcloud_height", "side_point_angle", "base_link_frame"},
    {rclcpp::ParameterEventsFilter::EventType::CHANGED});
  if (!filter.get_events().empty())
  {
    reconfigurePointCloud();
  }
}

void Bumper2PcNode::reconfigurePointCloud()
{
  // Lateral points x/y coordinates; we need to store float values to memcopy later
  float radius = static_cast<float>(get_parameter("pointcloud_radius").get_value<double>());
  float angle = static_cast<float>(get_parameter("side_point_angle").get_value<double>());
  p_side_x_ = + radius*sin(angle); // angle degrees from vertical
  p_side_y_ = + radius*cos(angle); // angle degrees from vertical
  n_side_y_ = - radius*cos(angle); // angle degrees from vertical

  // Prepare constant parts of the pointcloud message to be  published
  pointcloud_.header.frame_id = get_parameter("base_link_frame").get_value<std::string>();

  float height = static_cast<float>(this->get_parameter("pointcloud_height").get_value<double>());

  // z: constant elevation from base frame
  memcpy(&pointcloud_.data[0 * pointcloud_.point_step + pointcloud_.fields[2].offset], &height, sizeof(float));
  memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[2].offset], &height, sizeof(float));
  memcpy(&pointcloud_.data[2 * pointcloud_.point_step + pointcloud_.fields[2].offset], &height, sizeof(float));

  RCLCPP_INFO(get_logger(), "Bumper/cliff pointcloud configured at distance %f and height %f from base frame", radius, height);
}

Bumper2PcNode::Bumper2PcNode(const rclcpp::NodeOptions & options) : rclcpp::Node("kobuki_bumper2pc", options),
      P_INF_X(+100*sin(0.34906585)),
      P_INF_Y(+100*cos(0.34906585)),
      N_INF_Y(-100*cos(0.34906585)),
      ZERO(0.0), prev_bumper_(0), prev_cliff_(0)
{
  // Bumper/cliff pointcloud distance to base frame; should be something like the robot radius plus
  // costmap resolution plus an extra to cope with robot inertia. This is a bit tricky parameter: if
  // it's too low, costmap will ignore this pointcloud (the robot footprint runs over the hit obstacle),
  // but if it's too big, hit obstacles will be mapped too far from the robot and the navigation around
  // them will probably fail.

  declare_parameter("pointcloud_radius", 0.25);
  declare_parameter("pointcloud_height", 0.04);
  declare_parameter("side_point_angle", 0.34906585);
  declare_parameter("base_link_frame", "base_link");

  // Prepare constant parts of the pointcloud message to be published
  pointcloud_.width  = 3;
  pointcloud_.height = 1;
  pointcloud_.fields.resize(3);

  // Set x/y/z as the only fields
  pointcloud_.fields[0].name = "x";
  pointcloud_.fields[1].name = "y";
  pointcloud_.fields[2].name = "z";

  int offset = 0;
  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < pointcloud_.fields.size(); ++d, offset += 4)
  {
    pointcloud_.fields[d].count    = 1;
    pointcloud_.fields[d].offset   = offset;
    pointcloud_.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  pointcloud_.point_step = offset;
  pointcloud_.row_step   = pointcloud_.point_step * pointcloud_.width;

  pointcloud_.data.resize(3 * pointcloud_.point_step);
  pointcloud_.is_bigendian = false;
  pointcloud_.is_dense     = true;

  // Bumper/cliff "points" fix coordinates (the others depend on sensor activation/deactivation)

  // y: always 0 for central bumper
  memcpy(&pointcloud_.data[1 * pointcloud_.point_step + pointcloud_.fields[1].offset], &ZERO, sizeof(float));

  // Prepare configurable parts of the pointcloud message to be published
  reconfigurePointCloud();

  pointcloud_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
  core_sensor_sub_ = create_subscription<kobuki_ros_interfaces::msg::SensorState>("core_sensors", rclcpp::QoS(10), std::bind(&Bumper2PcNode::coreSensorCB, this, std::placeholders::_1));

  parameter_subscription_ = rclcpp::AsyncParametersClient::on_parameter_event(
    this->get_node_topics_interface(),
    std::bind(&Bumper2PcNode::onParameterEvent, this, std::placeholders::_1));
}

} // namespace kobuki_bumper2pc

RCLCPP_COMPONENTS_REGISTER_NODE(kobuki_bumper2pc::Bumper2PcNode)
