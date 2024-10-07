#!/bin/bash
docker run --rm \
  --net=host \
  -e ROS_DISTRO=humble \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///cyclonedds.xml \
  --name zenoh \
  -v ~/aichallenge-2024/vehicle/zenoh.json5:/zenoh.json5 \
  -v ~/aichallenge-2024/cyclonedds.xml:/cyclonedds.xml \
  eclipse/zenoh-bridge-ros2dds:latest \
  -c /zenoh.json5