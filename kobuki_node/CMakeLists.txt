cmake_minimum_required(VERSION 3.5)
project(kobuki_node)

find_package(ament_cmake_ros REQUIRED)
find_package(angles REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(ecl_build REQUIRED)
find_package(ecl_errors REQUIRED)
find_package(ecl_exceptions REQUIRED)
find_package(ecl_geometry REQUIRED)
find_package(ecl_linear_algebra REQUIRED)
find_package(ecl_sigslots REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_core REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

ecl_enable_cxx14_compiler()
ecl_enable_cxx_warnings()

include_directories(include)

add_library(kobuki_ros SHARED
  src/diagnostics.cpp
  src/kobuki_ros.cpp
  src/odometry.cpp
)
ament_target_dependencies(kobuki_ros
  "angles"
  "diagnostic_msgs"
  "diagnostic_updater"
  "geometry_msgs"
  "kobuki_core"
  "kobuki_ros_interfaces"
  "nav_msgs"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
  "tf2"
  "tf2_ros"
)
target_link_libraries(kobuki_ros
  ecl_errors::ecl_errors
  ecl_exceptions::ecl_exceptions
  ecl_geometry::ecl_geometry
  ecl_linear_algebra::ecl_linear_algebra
  ecl_sigslots::ecl_sigslots
)

rclcpp_components_register_node(kobuki_ros
  PLUGIN "kobuki_node::KobukiRos"
  EXECUTABLE kobuki_ros_node)

install(TARGETS kobuki_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
