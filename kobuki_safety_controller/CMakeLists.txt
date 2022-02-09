cmake_minimum_required(VERSION 3.5)
project(kobuki_safety_controller)

find_package(ament_cmake_ros REQUIRED)
find_package(ecl_build REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

ecl_enable_cxx14_compiler()
ecl_enable_cxx_warnings()

include_directories(include)

add_library(kobuki_safety_controller SHARED src/safety_controller.cpp)
ament_target_dependencies(kobuki_safety_controller
  "geometry_msgs"
  "kobuki_ros_interfaces"
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
)

rclcpp_components_register_node(kobuki_safety_controller
  PLUGIN "kobuki_safety_controller::SafetyController"
  EXECUTABLE kobuki_safety_controller_node)

install(TARGETS kobuki_safety_controller
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
