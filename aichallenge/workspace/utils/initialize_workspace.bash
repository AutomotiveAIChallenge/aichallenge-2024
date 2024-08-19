#!/usr/bin/bash
WORKSPACE=$(readlink -f "$(dirname "$0")/..")
cd "${WORKSPACE}" || exit
vcs import --shallow --input depends.repos depends

touch depends/autoware.universe/localization/gyro_odometer/COLCON_IGNORE
touch depends/autoware.universe/sensing/imu_corrector/COLCON_IGNORE

# shellcheck disable=SC2046
rosdep install --ignore-src --from-paths $(colcon list --paths-only --packages-up-to aichallenge_launch)
