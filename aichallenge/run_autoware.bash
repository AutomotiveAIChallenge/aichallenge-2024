#!/bin/bash

# shellcheck disable=SC1091
source /aichallenge/autoware/install/setup.bash
sudo ip link set multicast on lo
ros2 launch aichallenge_system_launch aichallenge_system.launch.xml
