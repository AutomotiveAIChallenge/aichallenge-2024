cmake_minimum_required(VERSION 3.8)
project(vehicle_state_culculator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(vehicle_state_culculator_node
  src/vehicle_state_culculator.cpp
)

ament_auto_package(INSTALL_TO_SHARE
)
