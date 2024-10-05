#!/bin/sh

# Check if an IP address is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <ip_address>"
    echo "Example: $0 192.168.1.100"
    exit 1
fi

# Store the provided IP address
IP_ADDRESS=$1
#!/bin/bash
docker run --rm \
  --net=host \
  -e ROS_DISTRO=humble \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///cyclonedds.xml \
  --name zenoh \
  -v ~/aichallenge/test_docker/aichallenge-2024/vehicle/zenoh.json5:/zenoh.json5 \
  -v ~/aichallenge-2024/cyclonedds.xml:/cyclonedds.xml \
  eclipse/zenoh-bridge-ros2dds \
  -e tcp/"$IP_ADDRESS":7447