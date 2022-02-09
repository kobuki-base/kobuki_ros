cmake_minimum_required(VERSION 3.5)

project(kobuki_bumper2pc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(kobuki_bumper2pc src/kobuki_bumper2pc.cpp)
target_include_directories(kobuki_bumper2pc PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(kobuki_bumper2pc
  "kobuki_ros_interfaces"
  "rcl_interfaces"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
)

rclcpp_components_register_node(kobuki_bumper2pc
  PLUGIN "kobuki_bumper2pc::Bumper2PcNode"
  EXECUTABLE kobuki_bumper2pc_node)

install(TARGETS kobuki_bumper2pc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
