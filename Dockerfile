FROM ros:melodic

RUN apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get -y install \
    ros-melodic-cv-bridge \
    ros-melodic-tf \
    python-pip python-matplotlib python-pandas \
  && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/utiasSTARS/pykitti.git && \
    cd pykitti && \
    git checkout d3e1bb81676e831886726cc5ed79ce1f049aef2c && \
    python setup.py install

COPY . /kitti2bag

RUN cd kitti2bag && \
    python setup.py install

WORKDIR /data

ENTRYPOINT ["/kitti2bag/docker_entrypoint.sh"]

