#!/bin/bash

if [[ ${1} == "clean" ]]; then
    echo "clean build"
    rm -r ./workspace/build/* ./workspace/install/*
fi

cd ./workspace || exit
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
