cmake_minimum_required(VERSION 3.5)
project(kobuki_auto_docking)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(ecl_linear_algebra REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_core REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

ecl_enable_cxx14_compiler()
ecl_enable_cxx_warnings()

add_library(kobuki_auto_docking src/auto_docking_ros.cpp)
target_include_directories(kobuki_auto_docking PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(kobuki_auto_docking
  "geometry_msgs"
  "kobuki_core"
  "kobuki_ros_interfaces"
  "message_filters"
  "nav_msgs"
  "rcl_interfaces"
  "rclcpp"
  "rclcpp_components"
  "rclcpp_action"
  "std_msgs"
  "tf2_geometry_msgs"
  "tf2_ros"
)
target_link_libraries(kobuki_auto_docking
  ecl_linear_algebra::ecl_linear_algebra
)

rclcpp_components_register_node(kobuki_auto_docking
  PLUGIN "kobuki_auto_docking::AutoDockingROS"
  EXECUTABLE kobuki_auto_docking_node)

install(TARGETS kobuki_auto_docking
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/activate.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
