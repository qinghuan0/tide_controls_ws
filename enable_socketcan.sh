#!/bin/bash
if [ "$1" == "all" ]; then
    ip link set can0 up type can bitrate 1000000
    ip link set can1 up type can bitrate 1000000
else 
    ip link set $1 up type can bitrate 1000000
fi
