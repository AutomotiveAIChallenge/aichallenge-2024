#!/usr/bin/bash

source /aichallenge/workspace/install/setup.bash
ros2 bag play /aichallenge/workspace/src/0920_demo_interface_only -l --clock &
ros2 launch aichallenge_system_launch aichallenge_system.launch.xml simulation:=false use_sim_time:=true run_rviz:=true is_rosbag:=true
