#!/usr/bin/bash

source install/setup.bash

ros2 launch aichallenge_launch aichallenge_submit.launch.xml \
use_sim_time:=false \
launch_vehicle_interface:=true
