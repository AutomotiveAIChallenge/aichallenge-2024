#!/usr/bin/bash

source install/setup.bash

ros2 launch aichallenge_system_launch aichallenge_system.launch.xml simulation:=false use_sim_time:=true run_rviz:=true
