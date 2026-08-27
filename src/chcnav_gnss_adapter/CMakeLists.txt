cmake_minimum_required(VERSION 3.8)
project(chcnav_gnss_adapter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(msg_interfaces REQUIRED)
find_package(gnss_comm REQUIRED)

add_executable(chcnav_to_gnss src/chcnav_to_gnss.cpp)
ament_target_dependencies(chcnav_to_gnss rclcpp msg_interfaces gnss_comm)
install(TARGETS chcnav_to_gnss DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
