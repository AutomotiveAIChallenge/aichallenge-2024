#!/bin/bash

# shellcheck disable=SC2086
rocker --x11 --devices /dev/dri --env ROS_DISTRO=humble --user --net host --privileged --name zenoh --volume aichallenge:/aichallenge -- "aichallenge-2024-dev-${USER}" zenoh-bridge-ros2dds -c /vehicle/zenoh.json5
