#!/bin/bash
# Usage: ./connect_zenoh.bash <target-vehicle (A1 to A8)>
set -e
SCRIPT_DIR=$(readlink -f "$(dirname "$0")")
IP_ADDR=$(python3 "${SCRIPT_DIR}/scan_ip_addr.py" $1)
zenoh-bridge-ros2dds -e "tcp/${IP_ADDR}:7447"
