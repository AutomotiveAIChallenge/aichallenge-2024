cmake_minimum_required(VERSION 3.5)
project(multi_layered_costmap_visualizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(multi_layered_costmap_visualizer SHARED
  src/multi_layered_costmap_visualizer.cpp
)

rclcpp_components_register_node(multi_layered_costmap_visualizer
  PLUGIN "multi_layered_costmap_visualizer::MultiLayeredCostmapVisualizer"
  EXECUTABLE multi_layered_costmap_visualizer_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)