ARG ROS_DISTRO=kinetic
FROM ros:${ROS_DISTRO}-ros-core

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update && apt-get -y upgrade \
  && apt-get -y install \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-opencv3 \
    ros-${ROS_DISTRO}-tf \
    python-pip python-matplotlib \
  && apt-get -y autoremove && apt-get -y clean && rm -rf /var/lib/apt/lists/*
# Upgrade numpy for pykitti's pandas requirement
RUN python -m pip install --upgrade pip setuptools numpy
COPY . /kitti2bag
RUN pip install /kitti2bag

# A dirty hack to fix pykitti's Python 2 incompatibility
RUN sed  -i 's/FileNotFoundError/IOError/g' /usr/local/lib/python2.7/dist-packages/pykitti/*.py

WORKDIR /data

ENTRYPOINT ["/kitti2bag/docker_entrypoint.sh"]

