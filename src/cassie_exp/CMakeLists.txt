cmake_minimum_required(VERSION 3.10)

project(cassie_exp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} include)

add_library(MheSrb src/MheSrb.cpp)
target_link_libraries(MheSrb OsqpEigen::OsqpEigen)

add_library(DecentralEst src/DecentralEst.cpp)
target_link_libraries(DecentralEst OsqpEigen::OsqpEigen MheSrb)

# realtime_est
add_library(EstSub src/EstSub.cpp)
target_link_libraries(EstSub DecentralEst)
ament_target_dependencies(EstSub rclcpp) 

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
