cmake_minimum_required(VERSION 3.5)

project(kobuki_random_walker)

find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(kobuki_random_walker src/random_walker.cpp)
target_include_directories(kobuki_random_walker PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(kobuki_random_walker
  "geometry_msgs"
  "kobuki_ros_interfaces"
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
)

rclcpp_components_register_node(kobuki_random_walker
  PLUGIN "kobuki_random_walker::RandomWalkerNode"
  EXECUTABLE kobuki_random_walker_node)

install(TARGETS kobuki_random_walker
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
