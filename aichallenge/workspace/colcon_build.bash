#!/usr/bin/bash
MAKEFLAGS="-j4 -l 3.0" colcon build --symlink-install --packages-up-to aichallenge_launch --cmake-args -DCMAKE_BUILD_TYPE=Release
