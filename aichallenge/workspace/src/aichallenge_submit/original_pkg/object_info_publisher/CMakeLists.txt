cmake_minimum_required(VERSION 3.8)
project(object_info_publisher)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)

add_executable(object_info_publisher_node src/object_info_publisher_node.cpp)

ament_target_dependencies(object_info_publisher_node rclcpp std_msgs autoware_auto_perception_msgs)

install(TARGETS
  object_info_publisher_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
