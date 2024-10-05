cmake_minimum_required(VERSION 3.8)
project(csv_path_changer_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTrajectory.srv"
  DEPENDENCIES autoware_auto_planning_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
