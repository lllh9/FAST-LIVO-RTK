cmake_minimum_required(VERSION 3.5)
project(chcnav)

message(STATUS "---------------------------")
message(STATUS "-- USE CHCNAV ROS DRIVER --")
message(STATUS "---------------------------")

set(CMAKE_BUILD_TYPE Debug)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(autoware_sensing_msgs REQUIRED)

#find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(msg_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
    include
    ${ament_INCLUDE_DIRS}
  
)

# debug flags
option(NTRIP_SERVER_INCLUDE "option for ntrip_server" ON)

# hc_msg_parser
include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node)
include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/hc_msg_parser)

if(APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
endif()

if(UNIX AND NOT APPLE)
    # If Linux, add rt and pthread
    set(rt_LIBRARIES rt)
    set(pthread_LIBRARIES pthread)
    #catkin_package(
        #LIBRARIES ${PROJECT_NAME}
        #INCLUDE_DIRS include
        #DEPENDS rt pthread
    #)
else()
    # Otherwise normal call
    #catkin_package(
        #LIBRARIES ${PROJECT_NAME}
        #INCLUDE_DIRS include
    #)
endif()

set(SERIAL_LIB_SRCS
    ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/src/serial.cc
    ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/include/serial/serial.h
    ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/include/serial/v8stdint.h
)
if(APPLE)
    # If OSX
    list(APPEND SERIAL_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/src/impl/unix.cc)
    list(APPEND SERIAL_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # If unix
    list(APPEND SERIAL_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/src/impl/unix.cc)
    list(APPEND SERIAL_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/src/impl/list_ports/list_ports_linux.cc)
else()
    # If windows
    list(APPEND SERIAL_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/src/impl/win.cc)
    list(APPEND SERIAL_LIB_SRCS ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/src/impl/list_ports/list_ports_win.cc)
endif()

include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common/serial/include)

include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector)
include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/udp_common)
include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/tcp_common)
include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/file_common)
include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/serial_common)
include_directories(${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/device_connector/can_common)

include_directories(
    PROJECT_SOURCE_DIR
    ${chcnav}/src/chcnav/include
)

set(CHCNAV_MSG_PARSER_SOURCES
        ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/hc_msg_parser/hc_msg_parser.c
        ${SERIAL_LIB_SRCS}
)

# hc_msg_processer
include_directories(${PROJECT_SOURCE_DIR}/src/hc_cgi_protocol_process_node)
set(CHCNAV_MSG_PROCESSER_SOURCES ${PROJECT_SOURCE_DIR}/src/hc_cgi_protocol_process_node/hc_cgi_protocol.c)

# a swas server class
if (NTRIP_SERVER_INCLUDE)
    include_directories(${PROJECT_SOURCE_DIR}/src/ntrip_server_node)
    set(CHCNAV_NTRIP_SOURCES ${PROJECT_SOURCE_DIR}/src/ntrip_server_node/swas_server.cpp)
    set(CHCNAV_ROS_LIBS swas_sdk)

    message(STATUS "architecture ${CMAKE_HOST_SYSTEM_PROCESSOR}")

    if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
        link_directories(${PROJECT_SOURCE_DIR}/lib/x86_64)
    elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
        link_directories(${PROJECT_SOURCE_DIR}/lib/aarch64)
    endif()

    set(CHCNAV_NODE_SOURCES
        ${PROJECT_SOURCE_DIR}/src/ntrip_server_node/NtripServerLaunchNode.cpp
        ${CHCNAV_NODE_SOURCES}
    )

endif()


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


# 设置依赖库
set(CHCNAV_ROS_LIBS ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBA_INIT} ${CHCNAV_ROS_LIBS} pthread dl)

# lib file 此处这样添加是为了输出日志
foreach(SINGLE_SOURCE_FILE ${CHCNAV_SERIAL_SOURCES} ${CHCNAV_NTRIP_SOURCES} ${CHCNAV_MSG_PARSER_SOURCES} ${CHCNAV_MSG_PROCESSER_SOURCES})
    message(STATUS "add ${SINGLE_SOURCE_FILE}")
    set(CHCNAV_ROS_SOURCES ${SINGLE_SOURCE_FILE} ${CHCNAV_ROS_SOURCES})
endforeach(SINGLE_SOURCE_FILE)

# node
set(CHCNAV_NODE_SOURCES
    ${PROJECT_SOURCE_DIR}/src/hc_cgi_protocol_process_node/HcCgiProtocolProcessNode.cpp
    ${PROJECT_SOURCE_DIR}/src/hc_msg_parser_node/HcMsgParserLaunchNode.cpp
    ${PROJECT_SOURCE_DIR}/src/demo/ChcnavFixDemo.cpp
    ${PROJECT_SOURCE_DIR}/src/demo/TimeUniformityNode.cpp
    ${PROJECT_SOURCE_DIR}/src/demo/RecordMsgToFile.cpp
    ${CHCNAV_NODE_SOURCES}
)

foreach(SINGLE_NODE_FILE ${CHCNAV_NODE_SOURCES} )
    message(STATUS "node file ${SINGLE_NODE_FILE} detected")
    STRING(REGEX REPLACE ".+/(.+)\\..*" "\\1" NODE_NAME ${SINGLE_NODE_FILE})
    message(STATUS "node ${NODE_NAME}")

    # node name is made of filename and _node
    add_executable(${NODE_NAME} ${SINGLE_NODE_FILE} ${CHCNAV_ROS_SOURCES})
    #add_dependencies(${NODE_NAME} ${PROJECT_NAME}_generate_messages_cpp)
    ament_target_dependencies(${NODE_NAME} rclcpp std_msgs msg_interfaces geometry_msgs tf2 tf2_ros sensor_msgs tf2_geometry_msgs autoware_sensing_msgs) 

    target_link_libraries(${NODE_NAME} ${CHCNAV_ROS_LIBS})

    install(TARGETS ${NODE_NAME}
        DESTINATION lib/${PROJECT_NAME})

endforeach(SINGLE_NODE_FILE)


ament_package()

message(STATUS "---------------------------")
message(STATUS "-- END CHCNAV ROS DRIVER --")
message(STATUS "---------------------------")

