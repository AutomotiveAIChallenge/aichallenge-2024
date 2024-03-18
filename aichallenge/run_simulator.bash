#!/bin/bash

# shellcheck disable=SC1091
source /aichallenge/autoware/install/setup.bash
sudo ip link set multicast on lo
/aichallenge/simulator/simulator.bash
