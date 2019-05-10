#!/bin/bash
set -e

# unset $@ since setup.bash catches some args like -h
args="$@"
set --

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "kitti2bag $args"
