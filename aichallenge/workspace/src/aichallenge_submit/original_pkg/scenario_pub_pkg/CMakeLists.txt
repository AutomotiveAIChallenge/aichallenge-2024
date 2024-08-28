cmake_minimum_required(VERSION 3.8)
project(scenario_pub_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tier4_planning_msgs REQUIRED)

add_executable(scenario_pub_node src/scenario_pub_node.cpp)
ament_target_dependencies(scenario_pub_node rclcpp std_msgs tier4_planning_msgs)
install(TARGETS scenario_pub_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
