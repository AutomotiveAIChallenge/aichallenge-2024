cmake_minimum_required(VERSION 3.14)
project(aichallenge_awsim_adapter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/actuation_cmd_converter.cpp
  src/sensor_converter.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ActuationCmdConverter"
  EXECUTABLE actuation_cmd_converter
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "SensorConverter"
  EXECUTABLE sensor_converter
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
