cmake_minimum_required(VERSION 3.8)
project(lateral_pure_pursuit)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(lateral_pure_pursuit_node
  src/lateral_pure_pursuit.cpp
)

ament_auto_package()