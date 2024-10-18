#!/bin/bash

sudo sysctl -w net.core.rmem_max=10485760
sudo sysctl -w net.core.rmem_default=10485760
sudo ip link set lo multicast on