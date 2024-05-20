#!/bin/bash

# Move working directory
OUTPUT_DIRECTORY=$(date +%Y%m%d-%H%M%S)
cd /output || exit
mkdir "$OUTPUT_DIRECTORY"
cd "$OUTPUT_DIRECTORY" || exit

# shellcheck disable=SC1091
source /aichallenge/autoware/install/setup.bash
sudo ip link set multicast on lo
sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null

# Start AWSIM
echo "Start AWSIM"
/aichallenge/simulator/AWSIM.x86_64 >/dev/null &
PID_AWSIM=$!
sleep 20

# Move AWSIM Window To Left
wmctrl -l

wmctrl -r "AWSIM" -e 0,0,0,960,1043

# Start Autoware
echo "Start Autoware"
ros2 launch aichallenge_system_launch aichallenge_system.launch.xml >autoware.log 2>&1 &
PID_AUTOWARE=$!
sleep 10

# Start recording rosbag
echo "Start rosbag"
ros2 bag record -a -o rosbag2_autoware >/dev/null 2>&1 &
PID_ROSBAG=$!
sleep 5

# Start recording rviz2 (TODO: This will not wait if there is no service)
echo "Start screen capture"
until (ros2 service type /debug/service/capture_screen >/dev/null); do
    sleep 5
done
ros2 service call /debug/service/capture_screen std_srvs/srv/Trigger >/dev/null
sleep 5

# Start driving and wait for the simulation to finish
echo "Waiting for the simulation"
ros2 service call /localization/trigger_node std_srvs/srv/SetBool '{data: true}' >/dev/null
wait $PID_AWSIM

# Stop recording rviz2
echo "Stop screen capture"
ros2 service call /debug/service/capture_screen std_srvs/srv/Trigger >/dev/null
sleep 10

# Stop recording rosbag
echo "Stop rosbag"
kill $PID_ROSBAG
wait $PID_ROSBAG

# Stop Autoware
echo "Stop Autoware"
kill $PID_AUTOWARE
wait $PID_AUTOWARE

# Convert result
echo "Convert result"
python3 /aichallenge/autoware/src/aichallenge_system/script/result-converter.py 60 11

# Compress rosbag
echo "Compress rosbag"
tar -czf rosbag2_autoware.tar.gz rosbag2_autoware
rm -rf rosbag2_autoware
