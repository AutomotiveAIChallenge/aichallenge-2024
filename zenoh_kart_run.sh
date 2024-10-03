#!/bin/sh

docker run --rm --net=host -e ROS_DISTRO=humble -v "$(pwd)"/zenoh.json5:/zenoh.json5 eclipse/zenoh-bridge-ros2dds -c /zenoh.json5
