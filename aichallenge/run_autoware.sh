#!/bin/bash

sudo ip link set multicast on lo

source ./autoware/install/setup.bash
rm -f result.json
ros2 launch aichallenge_system_launch aichallenge_system.launch.xml
