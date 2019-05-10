ARG ROS_DISTRO=kinetic
FROM ros:${ROS_DISTRO}-perception

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get -y update \
  # && apt-get -y upgrade \
  && apt-get -y install --no-install-recommends \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-tf \
    python-pip \
  && apt-get -y autoremove && apt-get -y clean && rm -rf /var/lib/apt/lists/*
# setup.py will fail on older Ubuntu distros if pip and setuptools are not updated
RUN python -m pip install --upgrade pip setuptools \
  && pip install --upgrade \
    # Upgrade numpy for pykitti's pandas requirement
    numpy \
    # Use the development version since a Python 2 incompatibility is not yet fixed in 0.3.1
    git+https://github.com/utiasSTARS/pykitti.git
COPY . /kitti2bag
RUN pip install /kitti2bag

WORKDIR /data

ENTRYPOINT ["/kitti2bag/docker_entrypoint.sh"]
