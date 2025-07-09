#!/bin/bash

# Example
# ./docker_convert_odom.sh /mnt/data/kitti/kitti_odom/dataset /mnt/data/kitti/kitti_odom/dataset/sequences/output

TIME=2011_09_26
SEQUENCE=00
INPUT=${1:-"./data"}
OUTPUT=${2:-"./output"}

# Ensure output directory exists
# rm -rf $OUTPUT
mkdir -p $OUTPUT

# Example run command
docker run -t --rm --tty \
    -e LOCAL_USER_ID=$(id -u) \
    -e LOCAL_GROUP_ID=$(id -g) \
    -v $INPUT:/data \
    -v $OUTPUT:/output \
    kitti2bag:ros2-jazzy \
    bash -c "kitti2bag odom_gray /data -s ${SEQUENCE} -o /output"

ls -lath $OUTPUT
