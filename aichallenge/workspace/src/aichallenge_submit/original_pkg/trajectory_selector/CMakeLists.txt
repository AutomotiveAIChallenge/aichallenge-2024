cmake_minimum_required(VERSION 3.8)
project(trajectory_selector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(tier4_planning_msgs REQUIRED)

add_executable(trajectory_selector_node src/trajectory_selector_node.cpp)
ament_target_dependencies(trajectory_selector_node rclcpp tier4_planning_msgs autoware_auto_planning_msgs)
install(TARGETS trajectory_selector_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()


