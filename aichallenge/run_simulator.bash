#!/bin/bash
AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM_GPU

# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo
$AWSIM_DIRECTORY/AWSIM.x86_64
