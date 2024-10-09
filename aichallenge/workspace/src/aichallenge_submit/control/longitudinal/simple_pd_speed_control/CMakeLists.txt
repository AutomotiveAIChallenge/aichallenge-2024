cmake_minimum_required(VERSION 3.8)
project(simple_pd_speed_control)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(simple_pd_speed_control_node
  src/simple_pd_speed_control.cpp
)

ament_auto_package()