#!/bin/bash

export PATH="$PATH:/root/.local/bin"
export PATH="/usr/local/cuda/bin:$PATH"
export XDG_RUNTIME_DIR=/tmp/xdg
export RCUTILS_COLORIZED_OUTPUT=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# shellcheck disable=SC1091
source /aichallenge/autoware/install/setup.bash
sudo ip link set multicast on lo

# Move working directory
cd /output || exit

# Launch the simulator
echo "Launch AWSIM"
bash /aichallenge/simulator/simulator.bash &

# Waiting for the simulator to start up
sleep 3

# Launch Autoware
echo "Launch user Autoware code"
ros2 launch aichallenge_system_launch aichallenge_system.launch.xml >autoware.log 2>&1 &
ROSLAUNCH_PID=$!

# Waiting for Autoware to start up
sleep 3

# Start recording rosbag
rm -r rosbag2_autoware
ros2 bag record -a -o rosbag2_autoware &
ROSBAG_RECORD_PID=$!

# Waiting for screen capture (TODO: This will not wait if there is no service)
# echo "Waiting for screen capture"
# until (ros2 service type /debug/service/capture_screen); do
#     sleep 5
# done

# Start recording rviz2
# ros2 service call /debug/service/capture_screen std_srvs/srv/Trigger

# Waiting for the simulator results
# echo "Waiting for the simulator results"
# until [ -f ~/awsim-logs/result.json ]; do
#     sleep 5
# done

# Stop recording rviz2
# ros2 service call /debug/service/capture_screen std_srvs/srv/Trigger

# Waiting for the screen capture to finish
sleep 3

## Stop rosbag and Autoware to finish writing logs
kill $ROSBAG_RECORD_PID
kill $ROSLAUNCH_PID

# Waiting for the rosbag and logs
sleep 3

## Compress rosbag
tar -czf rosbag2_autoware.tar.gz rosbag2_autoware
sleep 3

## Copy the logs to output directory
echo "Generation of result.json is completed."
cp ~/awsim-logs/result.json /output
cp ~/awsim-logs/verbose_result.json /output
