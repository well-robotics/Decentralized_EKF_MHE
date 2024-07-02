cmake_minimum_required(VERSION 3.10)

project(go1_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(optitrack_broadcast REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(decentral_legged_est REQUIRED)

include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${decentral_legged_est_INCLUDE_DIRS}  
  include
)
# aux_source_directory(. DIR_SRCS)

add_library(code_gen src/Expressions/FL_foot.cc
src/Expressions/FR_foot.cc
src/Expressions/RL_foot.cc
src/Expressions/RR_foot.cc
src/Expressions/J_FL.cc
src/Expressions/J_FR.cc
src/Expressions/J_RL.cc
src/Expressions/J_RR.cc
)

# realtime_est
add_executable(sub src/go1Sub.cpp)
target_link_libraries(sub code_gen OsqpEigen::OsqpEigen)
ament_target_dependencies(sub rclcpp optitrack_broadcast sensor_msgs custom_msgs decentral_legged_est) 

install( TARGETS sub DESTINATION lib/${PROJECT_NAME}) 

install(DIRECTORY 
        launch 
        DESTINATION share/${PROJECT_NAME}/
)
install(FILES config/parameters_go1.yaml
  DESTINATION share/${PROJECT_NAME}/config)  

  if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
#MPCExample
# add_executable(mhe src/mheSRB.cpp)
# target_link_libraries(mhe OsqpEigen::OsqpEigen transforms forward_kinematics)
