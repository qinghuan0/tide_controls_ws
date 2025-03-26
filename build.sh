#!/bin/bash

if [ "$1" == "all" ]; then
    if [ "$2" == "--debug" ]; then
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    else
        colcon build --symlink-install 
    fi
else
    if [ "$2" == "--debug" ]; then
        colcon build --packages-up-to "$1" --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
    else
        colcon build --packages-up-to "$1" --symlink-install
    fi
fi
