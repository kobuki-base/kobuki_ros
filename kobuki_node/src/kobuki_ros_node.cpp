#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "kobuki_node/kobuki_ros.hpp"

int main(int argc, char** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<kobuki_node::KobukiRos>(rclcpp::NodeOptions()));

  rclcpp::shutdown();

  return 0;
}
