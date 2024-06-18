cmake_minimum_required(VERSION 3.10)

project(mhe_pogox_exp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(orbslam3_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(optitrack_broadcast REQUIRED)
find_package(communication REQUIRED)
find_package(custom_msgs REQUIRED)

# aux_source_directory(. DIR_SRCS)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} include)

add_library(spline src/BSpline_simple.cpp
                   src/Bezier_simple.cpp)

add_library(codegen src/J_Toe.cc
                   src/pToe.cc)

add_library(mheSrb src/mheSrb.cpp)
target_link_libraries(mheSrb OsqpEigen::OsqpEigen)

add_library(mheEst src/mheEst.cpp)
target_link_libraries(mheEst OsqpEigen::OsqpEigen mheSrb spline codegen)

# realtime_est
add_library(pogox_sub src/pogox_sub.cpp)
target_link_libraries(pogox_sub mheEst)
ament_target_dependencies(pogox_sub rclcpp sensor_msgs orbslam3_msgs optitrack_broadcast custom_msgs geometry_msgs nav_msgs tf2 tf2_ros communication) 

add_executable(sub src/pogox_sub_node.cpp)
target_link_libraries(sub pogox_sub)
ament_target_dependencies(sub rclcpp)

install( DIRECTORY include/ DESTINATION include)

install( TARGETS sub DESTINATION lib/${PROJECT_NAME}) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
#MPCExample
# add_executable(mhe src/mheSRB.cpp)
# target_link_libraries(mhe OsqpEigen::OsqpEigen transforms forward_kinematics)
