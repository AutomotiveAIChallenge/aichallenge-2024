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

# Launch the simulator
echo "Start AWSIM"
/aichallenge/simulator/AWSIM.x86_64 >/dev/null &
PID_AWSIM=$!

# Waiting for the simulator to start up
sleep 10

# Launch Autoware
echo "Start Autoware"
ros2 launch aichallenge_system_launch aichallenge_system.launch.xml >autoware.log 2>&1 &
PID_AUTOWARE=$!

# Waiting for Autoware to start up
sleep 10

# Start recording rosbag
echo "Start rosbag"
ros2 bag record -a -o rosbag2_autoware >/dev/null 2>&1 &
PID_ROSBAG=$!

# Waiting for screen capture (TODO: This will not wait if there is no service)
echo "Waiting for screen capture"
until (ros2 service type /debug/service/capture_screen >/dev/null); do
    sleep 5
done

# Start recording rviz2
echo "Start screen capture"
ros2 service call /debug/service/capture_screen std_srvs/srv/Trigger >/dev/null

echo "Start driving"
ros2 service call /localization/trigger_node std_srvs/srv/SetBool '{data: true}' >/dev/null

echo "Waiting for the simulator: $PID_AWSIM"
wait $PID_AWSIM

# Stop recording rviz2
echo "Stop screen capture"
ros2 service call /debug/service/capture_screen std_srvs/srv/Trigger >/dev/null

# Waiting for the screen capture to finish
sleep 10

## Stop rosbag and Autoware to finish writing logs
echo "Stop rosbag: $PID_ROSBAG"
kill $PID_ROSBAG
echo "Stop Autoware: $PID_AUTOWARE"
kill $PID_AUTOWARE

# Waiting for the rosbag and logs
sleep 10

## Compress rosbag
echo "Compress rosbag"
tar -czf rosbag2_autoware.tar.gz rosbag2_autoware
rm -rf rosbag2_autoware
sleep 10

#until [ -f result-details.json ]; do
#    sleep 5
#done
