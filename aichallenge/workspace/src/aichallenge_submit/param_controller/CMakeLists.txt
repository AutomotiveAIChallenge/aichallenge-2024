cmake_minimum_required(VERSION 3.8)
project(param_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(param_controller_node
  src/param_controller.cpp
)

ament_auto_package(INSTALL_TO_SHARE
)
