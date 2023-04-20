cmake_minimum_required(VERSION 2.8.3)

project(irp_sen_msgs)
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE RelWithDebInfo)
endif ()
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs sensor_msgs
)

add_message_files(
  DIRECTORY msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)
