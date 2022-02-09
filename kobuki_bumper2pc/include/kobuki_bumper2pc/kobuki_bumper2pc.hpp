/*
 * Copyright (c) 2013, Yujin Robot.
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
 * @file /include/kobuki_bumper2pc/kobuki_bumper2pc.hpp
 *
 * @brief Bumper/cliff to pointcloud nodelet class declaration.
 *
 * Publish bumpers and cliff sensors events as points in a pointcloud, so navistack can use them
 * for poor-man navigation. Implemented as a nodelet intended to run together with kobuki_node.
 *
 * @author Jorge Santos, Yujin Robot
 *
 **/

#ifndef _KOBUKI_BUMPER2PC_HPP_
#define _KOBUKI_BUMPER2PC_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <memory>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <kobuki_ros_interfaces/msg/sensor_state.hpp>

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace kobuki_bumper2pc
{

/**
 * @brief Bumper2PcNodelet class declaration
 */
class Bumper2PcNode final : public rclcpp::Node
{
public:
  explicit Bumper2PcNode(const rclcpp::NodeOptions & options);
  ~Bumper2PcNode() { }

private:
  const float P_INF_X;  // somewhere out of reach from the robot (positive x)
  const float P_INF_Y;  // somewhere out of reach from the robot (positive y)
  const float N_INF_Y;  // somewhere out of reach from the robot (negative y)
  const float ZERO;

  uint8_t prev_bumper_;
  uint8_t prev_cliff_;

  float p_side_x_;
  float p_side_y_;
  float n_side_y_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::SensorState>::SharedPtr core_sensor_sub_;

  sensor_msgs::msg::PointCloud2 pointcloud_;

  std::shared_ptr<rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent,
    std::allocator<void>>> parameter_subscription_;

  void onParameterEvent(
    std::shared_ptr<rcl_interfaces::msg::ParameterEvent> event);

  void reconfigurePointCloud();

  /**
   * @brief Core sensors state structure callback
   * @param msg incoming topic message
   */
  void coreSensorCB(const std::shared_ptr<kobuki_ros_interfaces::msg::SensorState> msg);
};

} // namespace kobuki_bumper2pc

#endif // _KOBUKI_BUMPER2PC_HPP_
