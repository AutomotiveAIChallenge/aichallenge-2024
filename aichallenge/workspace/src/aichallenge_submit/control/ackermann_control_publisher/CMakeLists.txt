cmake_minimum_required(VERSION 3.8)
project(ackermann_control_publisher)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(ackermann_control_publisher_node
  src/ackermann_control_publisher.cpp
)

ament_auto_package()