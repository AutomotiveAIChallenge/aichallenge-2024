#!/usr/bin/bash

source install/setup.bash

ros2 launch aichallenge_launch aichallenge_system.launch.xml simulation:=true
