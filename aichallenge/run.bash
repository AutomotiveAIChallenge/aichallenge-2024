#!/bin/bash
AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM_GPU


# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo
sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null

# Start rqt
echo "Start rqt"
rqt >/dev/null &
PID_RQT=$!


# Start AWSIM
echo "Start AWSIM"
$AWSIM_DIRECTORY/AWSIM.x86_64 >/dev/null &
PID_AWSIM=$!
sleep 10

# Start Autoware(sleep時間は任意の時間に変更)
echo "Start Autoware"
ros2 launch aichallenge_system_launch aichallenge_system.launch.xml >autoware.log 2>&1 &

PID_AUTOWARE=$!
sleep 15


# Start driving and wait for the simulation to finish
echo "Waiting for the simulation"
ros2 topic pub --once /control/control_mode_request_topic std_msgs/msg/Bool '{data: true}' >/dev/null
wait $PID_AWSIM
