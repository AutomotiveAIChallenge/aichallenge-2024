#!/bin/bash
# shellcheck disable=SC2086

rocker --x11 --devices /dev/dri --user --net host --privileged --name zenoh --volume aichallenge:/aichallenge -- "aichallenge-2024-dev-${USER}"  bash -c "zenoh-bridge-ros2dds -c /vehicle/zenoh.json5"
# rocker --x11 --devices /dev/dri --user --net host --privileged --name zenoh --volume aichallenge:/aichallenge -- "aichallenge-2024-dev-${USER}"  bash -c "ROS_DISTRO=humble zenoh-bridge-ros2dds -c /vehicle/zenoh.json5"
# rocker --x11 --devices /dev/dri --user --net host --privileged --name zenoh --volume ../aichallenge:/aichallenge -- "aichallenge-2024-dev-${USER}"  

