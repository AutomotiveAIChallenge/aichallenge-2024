#!/bin/bash
AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM

# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo
sudo sysctl -w net.core.rmem_max=2147483647 >/dev/null
$AWSIM_DIRECTORY/AWSIM.x86_64
