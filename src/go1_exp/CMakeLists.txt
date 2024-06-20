cmake_minimum_required(VERSION 3.10)

project(go1_exp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orbslam3_msgs REQUIRED)
find_package(optitrack_broadcast REQUIRED)

# aux_source_directory(. DIR_SRCS)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} include)

add_library(code_gen src/Expressions/FL_foot.cc
src/Expressions/FR_foot.cc
src/Expressions/RL_foot.cc
src/Expressions/RR_foot.cc
src/Expressions/J_FL.cc
src/Expressions/J_FR.cc
src/Expressions/J_RL.cc
src/Expressions/J_RR.cc
)

add_library(spline src/spline/BSpline_simple.cpp
                   src/spline/Bezier_simple.cpp)

add_library(MheSrb src/MheSrb.cpp)
target_link_libraries(MheSrb OsqpEigen::OsqpEigen)

add_library(DecentralEst src/DecentralEst.cpp)
target_link_libraries(DecentralEst OsqpEigen::OsqpEigen MheSrb spline code_gen)

# realtime_est
add_library(EstSub src/EstSub.cpp)
target_link_libraries(EstSub DecentralEst)
ament_target_dependencies(EstSub rclcpp orbslam3_msgs sensor_msgs geometry_msgs optitrack_broadcast) 

add_executable(sub src/EstSubNode.cpp)
target_link_libraries(sub EstSub)
ament_target_dependencies(sub rclcpp)

install( DIRECTORY include/ DESTINATION include)

install( TARGETS sub DESTINATION lib/${PROJECT_NAME}) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
#MPCExample
# add_executable(mhe src/mheSRB.cpp)
# target_link_libraries(mhe OsqpEigen::OsqpEigen transforms forward_kinematics)
