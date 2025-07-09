#!/bin/bash

# Example
# ./docker_convert_raw.sh /mnt/data/kitti /mnt/data/kitti/output

TIME=2011_09_26
SEQUENCE=0002
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
    bash -c "kitti2bag raw_synced /data -t ${TIME} -r ${SEQUENCE} -o /output"

ls -lath $OUTPUT
