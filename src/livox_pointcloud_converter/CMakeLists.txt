cmake_minimum_required(VERSION 3.8)
project(livox_pointcloud_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(livox_pointcloud_converter_node
  src/livox_pointcloud_converter_node.cpp
)
target_compile_features(livox_pointcloud_converter_node PUBLIC cxx_std_17)
ament_target_dependencies(livox_pointcloud_converter_node
  rclcpp
  sensor_msgs
)

add_executable(pointcloud2_to_custom_node
  src/pointcloud2_to_custom_node.cpp
)
target_compile_features(pointcloud2_to_custom_node PUBLIC cxx_std_17)
ament_target_dependencies(pointcloud2_to_custom_node
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

install(TARGETS
  livox_pointcloud_converter_node
  pointcloud2_to_custom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
