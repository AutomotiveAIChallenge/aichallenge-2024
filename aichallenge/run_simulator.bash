#!/bin/bash

# shellcheck disable=SC1091
source /aichallenge/workspace/install/setup.bash
sudo ip link set multicast on lo
/aichallenge/simulator/AWSIM.x86_64
