cmake_minimum_required(VERSION 3.8)
project(fast_livo)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Architecture-specific optimisation (mirrors the ROS 1 package).
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -ffast-math")
  else()
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mcpu=native -mtune=native -mfpu=neon -ffast-math")
  endif()
  add_definitions(-DARM_ARCH)
else()
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -funroll-loops")
  add_definitions(-DX86_ARCH)
endif()

# ROOT_DIR is used by the node to locate Log/ and scripts/ at runtime.
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Multithreading (capped at 4 like upstream).
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")
if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
  set(OPENMP_LIBS OpenMP::OpenMP_CXX)
else()
  set(OPENMP_LIBS "")
endif()

find_package(mimalloc QUIET)

# --- ament / ROS 2 dependencies --------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_srvs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(gnss_comm REQUIRED)
find_package(vikit_common REQUIRED)
find_package(vikit_ros REQUIRED)

# --- system (non-ament) dependencies ---------------------------------------
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread timer)
if(EXISTS "/usr/share/cmake/geographiclib")
  list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
endif()
find_package(GeographicLib REQUIRED)
find_package(GTSAM REQUIRED QUIET)

set(Sophus_LIBRARIES libSophus.so)

set(ament_dependencies
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
  std_srvs
  livox_ros_driver2
  gnss_comm
  vikit_common
  vikit_ros
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# --- libraries (one per subsystem, as upstream) ----------------------------
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(laser_mapping src/LIVMapper.cpp)
add_library(optimization src/optimization.cpp src/global_esikf.cpp)

set(FASTLIVO_LIBS vio lio pre imu_proc laser_mapping optimization)
foreach(lib ${FASTLIVO_LIBS})
  ament_target_dependencies(${lib} ${ament_dependencies})
  target_link_libraries(${lib}
    ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES}
    ${Boost_LIBRARIES} ${GeographicLib_LIBRARIES} gtsam ${OPENMP_LIBS})
endforeach()

# --- main executable -------------------------------------------------------
add_executable(fastlivo_mapping src/main.cpp)
ament_target_dependencies(fastlivo_mapping ${ament_dependencies})
target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  optimization
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  gtsam
  ${OPENMP_LIBS}
)
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

install(TARGETS fastlivo_mapping
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz_cfg
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
