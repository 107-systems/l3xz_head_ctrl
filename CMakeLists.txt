#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(l3xz_head_ctrl)
set(L3XZ_HEAD_CTRL_TARGET ${PROJECT_NAME}_node)
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_heartbeat REQUIRED)
find_package(ros2_loop_rate_monitor REQUIRED)
find_package(ros2_dynamixel_bridge REQUIRED)
find_package(mp-units REQUIRED)
#######################################################################################
include_directories(${L3XZ_HEAD_CTRL_TARGET}
  include
)
#######################################################################################
add_executable(${L3XZ_HEAD_CTRL_TARGET}
  src/Node.cpp
  src/main.cpp
)
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${L3XZ_HEAD_CTRL_TARGET} PRIVATE -Wall -Werror -Wextra -Wpedantic)
endif()
#######################################################################################
target_compile_features(${L3XZ_HEAD_CTRL_TARGET} PRIVATE cxx_std_20)
#######################################################################################
ament_target_dependencies(${L3XZ_HEAD_CTRL_TARGET} rclcpp geometry_msgs std_msgs ros2_heartbeat ros2_loop_rate_monitor ros2_dynamixel_bridge mp-units)
#######################################################################################
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
#######################################################################################
install(TARGETS
  ${L3XZ_HEAD_CTRL_TARGET}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
#######################################################################################
ament_package()
#######################################################################################
