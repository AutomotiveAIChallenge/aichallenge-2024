#!/bin/bash

sudo ip link set multicast on lo

source ./autoware/install/setup.bash
/aichallenge/AWSIM/AWSIM.x86_64
