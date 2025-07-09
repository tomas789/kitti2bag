#!/bin/bash

# Example
# ./docker_interact.sh /mnt/data/kitti/kitti_odom/dataset /mnt/data/kitti/kitti_odom/dataset/sequences/output

# Inside container example:
# rm -rf /output/*
# kitti2bag raw_synced /data -t 2011_09_26 -r 0001 -o /output

INPUT=${1:-"./data"}
OUTPUT=${2:-"./output"}

# Ensure output directory exists
mkdir -p $OUTPUT

# Run in interactive bash terminal
docker run -it --rm --tty \
    --name kitti2bag-ros2-jazzy \
    --ipc=host --privileged \
    -e LOCAL_USER_ID=$(id -u) \
    -e LOCAL_GROUP_ID=$(id -g) \
    -v $INPUT:/data \
    -v $OUTPUT:/output \
    kitti2bag:ros2-jazzy \
    bash
