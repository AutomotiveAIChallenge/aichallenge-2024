#!/bin/bash

# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo

ros2 launch aichallenge_system_launch aichallenge_system.launch.xml simulation:=false use_sim_time:=true run_rviz:=true
