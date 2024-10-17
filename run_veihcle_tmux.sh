#!/bin/bash

bash stop_vehicle_tmux.sh

sudo sysctl -w net.core.rmem_max=10485760
sudo sysctl -w net.core.rmem_default=10485760
sudo ip link set lo multicast on

tmux new-session \; source-file run_script.tmux
