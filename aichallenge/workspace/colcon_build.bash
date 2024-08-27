#!/usr/bin/bash
MAKEFLAGS=-j4 colcon build --symlink-install --packages-up-to aichallenge_launch --executor sequential
