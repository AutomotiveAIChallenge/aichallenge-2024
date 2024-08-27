cmake_minimum_required(VERSION 3.5)
project(booars_dummy_perception_publisher)

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

include_directories(include)

ament_auto_add_library(booars_dummy_perception_publisher SHARED
  src/dummy_objects_publisher.cpp
)

rclcpp_components_register_node(booars_dummy_perception_publisher
  PLUGIN "booars_dummy_perception_publisher::DummyObjectsPublisher"
  EXECUTABLE dummy_objects_publisher
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()