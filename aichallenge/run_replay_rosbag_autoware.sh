#!/usr/bin/bash

source /aichallenge/workspace/install/setup.bash
ros2 bag play /aichallenge/workspace/rosbag/0920_demo_interface_only_02 --topics /control/command/actuation_cmd /control/command/control_cmd /sensing/gnss/pose_with_covariance /sensing/gnss/pose /sensing/imu/imu_raw /vehicle/status/actuation_status /vehicle/status/control_mode /vehicle/status/gear_status /vehicle/status/steering_status /vehicle/status/velocity_status --clock &
ros2 launch aichallenge_system_launch aichallenge_system.launch.xml simulation:=false use_sim_time:=true run_rviz:=true is_rosbag:=true
