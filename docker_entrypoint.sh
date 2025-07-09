#!/bin/bash
set -e

# Default to user 1000 if not specified
USER_ID=${LOCAL_USER_ID:-1000}
GROUP_ID=${LOCAL_GROUP_ID:-1000}

# Create a user and group with the specified IDs
groupadd -g $GROUP_ID -o kitti2bag 2>/dev/null || true
useradd --shell /bin/bash -u $USER_ID -g $GROUP_ID -o -c "" -m kitti2bag 2>/dev/null || true
export HOME=/home/kitti2bag

# Populate .bashrc for the new user to guarantee a good interactive environment
echo "source /opt/ros/jazzy/setup.bash" >> $HOME/.bashrc
echo "if [ -f /kitti2bag/install/setup.bash ]; then source /kitti2bag/install/setup.bash; fi" >> $HOME/.bashrc
echo "export PATH=/kitti2bag/install/kitti2bag/bin:\$PATH" >> $HOME/.bashrc

# Execute command as kitti2bag user
# The .bashrc will be automatically sourced for interactive shells
exec gosu kitti2bag "$@"
