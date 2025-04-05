#!/bin/bash

# shellcheck disable=SC1091
source "/aichallenge/workspace/install/setup.bash"
ros2 bag record -a -x "(/racing_kart/.*|/to_can_bus|/from_can_bus)"
