FROM ros:jazzy-ros-base

# Install ROS2 Jazzy dependencies and Python packages
RUN apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get -y install \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-pip \
    python3-rosdep \
    ros-jazzy-cv-bridge \
    ros-jazzy-rosbag2-py \
    ros-jazzy-sensor-msgs \
    ros-jazzy-tf-transformations \
    ros-jazzy-tf2-ros \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get -y install \
    gosu rsync tree vim wget zip \
  && rm -rf /var/lib/apt/lists/*

# Install Python dependencies (including tf_transformations via pip)
RUN pip3 install --no-cache-dir -v --break-system-packages \
  pykitti \
  progressbar2

# Copy the package
COPY . /kitti2bag
WORKDIR /kitti2bag

# Build the ROS2 package
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Install the console script wrapper
RUN cp /kitti2bag/bin/kitti2bag /usr/local/bin/kitti2bag && \
    chmod +x /usr/local/bin/kitti2bag

# Set up the workspace
WORKDIR /data

# Source ROS2 and the built package
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
  echo "source /kitti2bag/install/setup.bash" >> ~/.bashrc && \
  chmod +x /kitti2bag/docker_entrypoint.sh

ENTRYPOINT ["/kitti2bag/docker_entrypoint.sh"]
