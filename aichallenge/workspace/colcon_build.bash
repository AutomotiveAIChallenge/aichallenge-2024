#!/usr/bin/bash
colcon build --symlink-install --packages-up-to aichallenge_launch --parallel-workers 1 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
