cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path
set(ENV{PYTHONPATH} "/opt/ros/galactic/lib/python3.8/site-packages")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(NOT CMAKE_BUILD_TYPE) 
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE) 
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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_msgs REQUIRED)


find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

link_directories(
  include
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters
ORB_SLAM3 Pangolin)

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin geometry_msgs Eigen3)

add_executable(stereo-decentralized
  src/stereo-decentralized/stereo-pub.cpp
  src/stereo-decentralized/stereo-pub-node.cpp
)
ament_target_dependencies(stereo-decentralized rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin geometry_msgs custom_msgs Eigen3)

install(TARGETS mono rgbd stereo stereo-inertial stereo-decentralized 
  DESTINATION lib/${PROJECT_NAME})
install(FILES config/stereo-decentralized/RealSense_D455_640_480.yaml
  DESTINATION share/${PROJECT_NAME}/config)
install(FILES vocabulary/ORBvoc.txt
  DESTINATION share/${PROJECT_NAME}/vocabulary)  
# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()

