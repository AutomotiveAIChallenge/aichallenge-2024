#!/bin/bash
SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
docker run --rm \
    --net=host \
    -e ROS_DISTRO=humble \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///vehicle/cyclonedds.xml \
    -v "${SCRIPT_DIR}:/vehicle" \
    --name zenoh \
    eclipse/zenoh-bridge-ros2dds:latest -c /vehicle/zenoh.json5
