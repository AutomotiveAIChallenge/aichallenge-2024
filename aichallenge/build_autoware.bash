#!/bin/bash

if [[ ${1} == "clean" ]]; then
    echo "clean build"
    rm -r autoware/build/* autoware/install/*
fi

cd ./autoware || exit
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
