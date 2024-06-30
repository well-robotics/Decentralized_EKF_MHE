cmake_minimum_required(VERSION 3.8)
project(orien_est)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(communication REQUIRED)
find_package(optitrack_broadcast REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} include)

add_executable(sub src/orien_ekf.cpp)
# ament_target_dependencies(imu_ekf rclcpp sensor_msgs geometry_msgs Eigen3 communication optitrack_broadcast)
ament_target_dependencies(sub rclcpp sensor_msgs geometry_msgs Eigen3 optitrack_broadcast)

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
