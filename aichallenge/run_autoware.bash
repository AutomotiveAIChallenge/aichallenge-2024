#!/bin/bash

mode=${1}

case "${mode}" in
"awsim")
    opts="simulation:=true use_sim_time:=true run_rviz:=true"
    ;;
"vehicle")
    opts="simulation:=false use_sim_time:=false run_rviz:=false"
    ;;
"rosbag")
    opts="simulation:=false use_sim_time:=true run_rviz:=true"
    ;;
*)
    echo "invalid argument (use 'awsim' or 'vehicle' or 'rosbag')"
    exit 1
    ;;
esac

# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo

# shellcheck disable=SC2086
ros2 launch aichallenge_system_launch aichallenge_system.launch.xml ${opts}
