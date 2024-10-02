#!/bin/bash
AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM

# Kill all processes
source /aichallenge/pkill_all.sh

# Move working directory
OUTPUT_DIRECTORY=$(date +%Y%m%d-%H%M%S)
cd /output || exit
mkdir "$OUTPUT_DIRECTORY"
ln -nfs "$OUTPUT_DIRECTORY" latest
cd "$OUTPUT_DIRECTORY" || exit

# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo
sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null

# Start AWSIM
echo "Start AWSIM"
$AWSIM_DIRECTORY/AWSIM.x86_64 >/dev/null &
PID_AWSIM=$!
sleep 20

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

# Start recording rviz2
echo "Start screen capture"
until (ros2 service type /debug/service/capture_screen >/dev/null); do
    sleep 5
done

# Move windows
wmctrl -a "RViz" && wmctrl -r "RViz" -e 0,0,0,1920,1043
wmctrl -a "AWSIM" && wmctrl -r "AWSIM" -e 0,0,0,900,1043

ros2 service call /debug/service/capture_screen std_srvs/srv/Trigger >/dev/null
sleep 1

# Set initial pose
echo "Set initial pose"
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{ 
  header: {
    frame_id: 'map'
  },
  pose: {
    pose: {
      position: {
        x: 89633.29,
        y: 43127.57,
        z: 0.0
      },
      orientation: {
        x: 0.0,
        y: 0.0,
        z: 0.8778,
        w: 0.4788
      }
    }
  }
}" >/dev/null
sleep 1

# Start driving and wait for the simulation to finish
echo "Waiting for the simulation"
ros2 service call /control/control_mode_request autoware_auto_vehicle_msgs/srv/ControlModeCommand '{mode: 1}' >/dev/null
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
python3 /aichallenge/workspace/src/aichallenge_system/script/result-converter.py 60 11

# Compress rosbag
echo "Compress rosbag"
tar -czf rosbag2_autoware.tar.gz rosbag2_autoware
rm -rf rosbag2_autoware
