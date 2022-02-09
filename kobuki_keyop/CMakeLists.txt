cmake_minimum_required(VERSION 3.5)
project(kobuki_keyop)

find_package(ament_cmake_ros REQUIRED)
find_package(ecl_build REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

ecl_enable_cxx14_compiler()
ecl_enable_cxx_warnings()

include_directories(include)

add_library(kobuki_keyop SHARED src/keyop.cpp)
ament_target_dependencies(kobuki_keyop
  "geometry_msgs"
  "kobuki_ros_interfaces"
  "rclcpp"
  "rclcpp_components"
)

rclcpp_components_register_node(kobuki_keyop
  PLUGIN "kobuki_keyop::KeyOp"
  EXECUTABLE kobuki_keyop_node)

install(TARGETS kobuki_keyop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
