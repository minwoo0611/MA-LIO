cmake_minimum_required(VERSION 3.8)
project(ma_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# ── Multiprocessing core-count detection (from upstream) ──
message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processor number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN -DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN -DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

# ── ROS2 / ament dependencies ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
  message_filters
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include
)

# ── Custom message ──
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
)
ament_export_dependencies(rosidl_default_runtime)

# ── Node executable ──
add_executable(malio_mapping
  src/laserMapping.cpp
  src/BsplineSE3.cpp
  include/ikd-Tree/ikd_Tree.cpp
  src/preprocess.cpp
  src/parameters.cpp
)
target_include_directories(malio_mapping PRIVATE include)
target_link_libraries(malio_mapping ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(malio_mapping ${dependencies})

# Link the same-package generated Pose6D msg into the node.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(malio_mapping ${cpp_typesupport_target})

# ── Install ──
install(TARGETS malio_mapping
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
