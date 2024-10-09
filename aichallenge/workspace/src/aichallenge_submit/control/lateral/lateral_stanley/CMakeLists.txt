cmake_minimum_required(VERSION 3.8)
project(lateral_stanley)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(lateral_stanley_node
  src/lateral_stanley.cpp
)

ament_auto_package()