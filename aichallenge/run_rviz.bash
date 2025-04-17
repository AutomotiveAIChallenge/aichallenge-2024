#!/bin/bash
# shellcheck disable=SC1091

#!/bin/bash

mode="${1}"

case "${mode}" in
"awsim")
    opts=("use_sim_time:=true")
    ;;
"vehicle")
    opts=("use_sim_time:=false")
    ;;
*)
    echo "invalid argument (use 'awsim' or 'vehicle')"
    exit 1
    ;;
esac

source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo
sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null

source /aichallenge/workspace/install/setup.bash
rviz2 -d /aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/debug_sensing.rviz \
    -s /aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/fast.png \
    --ros-args --remap "${opts[@]}"
# rviz2 -d /aichallenge/workspace/src/aichallenge_system/aichallenge_system_launch/config/debug_sensing.rviz
