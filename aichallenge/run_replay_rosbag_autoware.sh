#!/usr/bin/bash

source /aichallenge/workspace/install/setup.bash
ros2 bag play /aichallenge/workspace/src/rosbag_data/0920_demo_interface_only_02 -l --clock &
ros2 launch aichallenge_system_launch aichallenge_system.launch.xml simulation:=false use_sim_time:=true run_rviz:=true is_rosbag:=true
