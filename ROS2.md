# Run instructions for ROS2

Here are the instructions how to run kitti2bag in ROS2 docker container. These are examples so please adopt the bash scripts according to your need.

## Build docker image

To build the ROS2 Jazzy docker environment, run the following.

```bash
./docker_build.sh
```

which basically does this:

```bash
docker build -t kitti2bag:ros2-jazzy .
```

### Example stdout

```
[+] Building 4.0s (15/15) FINISHED                                                                                docker:default
 => [internal] load build definition from Dockerfile                                                                        0.0s
 => => transferring dockerfile: 1.33kB                                                                                      0.0s
 => [internal] load metadata for docker.io/library/ros:jazzy-ros-base                                                       0.9s
 => [internal] load .dockerignore                                                                                           0.0s
 => => transferring context: 454B                                                                                           0.0s
 => [ 1/10] FROM docker.io/library/ros:jazzy-ros-base@sha256:026816c588cc465b3838721edd970a66827bd6a4618e49969d8e5dfa977c6  0.0s
 => [internal] load build context                                                                                           0.0s
 => => transferring context: 2.58kB                                                                                         0.0s
 => CACHED [ 2/10] RUN apt-get update   && DEBIAN_FRONTEND=noninteractive apt-get -y install     python3-colcon-common-ext  0.0s
 => CACHED [ 3/10] RUN apt-get update   && DEBIAN_FRONTEND=noninteractive apt-get -y install     gosu   && rm -rf /var/lib  0.0s
 => CACHED [ 4/10] RUN pip3 install --no-cache-dir -v --break-system-packages   pykitti   progressbar2                      0.0s
 => [ 5/10] COPY . /kitti2bag                                                                                               0.0s
 => [ 6/10] WORKDIR /kitti2bag                                                                                              0.1s
 => [ 7/10] RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"                                             2.0s
 => [ 8/10] RUN cp /kitti2bag/bin/kitti2bag /usr/local/bin/kitti2bag &&     chmod +x /usr/local/bin/kitti2bag               0.3s
 => [ 9/10] WORKDIR /data                                                                                                   0.1s 
 => [10/10] RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc &&   echo "source /kitti2bag/install/setup.bash" >> ~  0.3s 
 => exporting to image                                                                                                      0.2s 
 => => exporting layers                                                                                                     0.2s 
 => => writing image sha256:4cd2aa77c5c8736709a612afe767e5db4c19ac8ac5bfc250c7508b1bc6f8b087                                0.0s 
 => => naming to docker.io/library/kitti2bag:ros2-jazzy                                                                     0.0s
```

## Example of running docker container in interactive mode

```bash
./docker_interact.sh
```

Inside the docker container, you can run the following commands to convert kitti sequences to rosbag2 mcap file. Note that `kitti2bag` is functionally equivalent to `ros2 run kitti2bag kitti2bag`.

## KITTI sequence 0001 from 2011_09_26

### Convert kitti to mcap

```bash
kitti2bag -t 2011_09_26 -r 0001 raw_synced /data -o /output
```

#### Example stdout
```
Exporting static transformations
Exporting time dependent transformations
Exporting IMU
Exporting camera 0
| |     #                                                                                            | 107 Elapsed Time: 0:00:00
Exporting camera 1
| |     #                                                                                            | 107 Elapsed Time: 0:00:00
Exporting camera 2
| |           #                                                                                      | 107 Elapsed Time: 0:00:01
Exporting camera 3
| |           #                                                                                      | 107 Elapsed Time: 0:00:01
Exporting velodyne data
| |  #                                                                                               | 107 Elapsed Time: 0:00:00
```

### Print info about the converted mcap

```bash
ros2 bag info /output/kitti_2011_09_26_drive_0001_synced/
```

#### Example stdout
```
Files:             kitti_2011_09_26_drive_0001_synced_0.mcap
Bag size:          585.4 MiB
Storage id:        mcap
ROS Distro:        jazzy
Duration:          11.040485859s
Start:             Sep 26 2011 13:02:25.951199054 (1317042145.951199054)
End:               Sep 26 2011 13:02:36.991684913 (1317042156.991684913)
Messages:          1512
Topic information: Topic: /kitti/camera_color_left/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 108 | Serialization Format: cdr
                   Topic: /kitti/camera_color_left/image_raw | Type: sensor_msgs/msg/Image | Count: 108 | Serialization Format: cdr
                   Topic: /kitti/camera_color_right/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 108 | Serialization Format: cdr
                   Topic: /kitti/camera_color_right/image_raw | Type: sensor_msgs/msg/Image | Count: 108 | Serialization Format: cdr
                   Topic: /kitti/camera_gray_left/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 108 | Serialization Format: cdr
                   Topic: /kitti/camera_gray_left/image_raw | Type: sensor_msgs/msg/Image | Count: 108 | Serialization Format: cdr
                   Topic: /kitti/camera_gray_right/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 108 | Serialization Format: cdr
                   Topic: /kitti/camera_gray_right/image_raw | Type: sensor_msgs/msg/Image | Count: 108 | Serialization Format: cdr
                   Topic: /kitti/oxts/gps/fix | Type: sensor_msgs/msg/NavSatFix | Count: 108 | Serialization Format: cdr
                   Topic: /kitti/oxts/gps/vel | Type: geometry_msgs/msg/TwistStamped | Count: 108 | Serialization Format: cdr
                   Topic: /kitti/oxts/imu | Type: sensor_msgs/msg/Imu | Count: 108 | Serialization Format: cdr
                   Topic: /kitti/velo/pointcloud | Type: sensor_msgs/msg/PointCloud2 | Count: 108 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 108 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 108 | Serialization Format: cdr
Service:           0
Service information: 
```

## KITTI sequence 0002 from 2011_09_26

### Convert kitti to mcap

```bash
kitti2bag -t 2011_09_26 -r 0002 raw_synced /data -o /output
```

#### Example stdout
```
Exporting static transformations
Exporting time dependent transformations
Exporting IMU
Exporting camera 0
| |    #                                                                                              | 76 Elapsed Time: 0:00:00
Exporting camera 1
| |    #                                                                                              | 76 Elapsed Time: 0:00:00
Exporting camera 2
| |        #                                                                                          | 76 Elapsed Time: 0:00:00
Exporting camera 3
| |        #                                                                                          | 76 Elapsed Time: 0:00:00
Exporting velodyne data
| |  #                                                                                                | 76 Elapsed Time: 0:00:00
```

### Print info about the converted mcap

```bash
ros2 bag info /output/kitti_2011_09_26_drive_0002_synced/
```

#### Example stdout

```
Files:             kitti_2011_09_26_drive_0002_synced_0.mcap
Bag size:          417.2 MiB
Storage id:        mcap
ROS Distro:        jazzy
Duration:          7.841315984s
Start:             Sep 26 2011 13:02:44.317411899 (1317042164.317411899)
End:               Sep 26 2011 13:02:52.158727883 (1317042172.158727883)
Messages:          1078
Topic information: Topic: /kitti/camera_color_left/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 77 | Serialization Format: cdr
                   Topic: /kitti/camera_color_left/image_raw | Type: sensor_msgs/msg/Image | Count: 77 | Serialization Format: cdr
                   Topic: /kitti/camera_color_right/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 77 | Serialization Format: cdr
                   Topic: /kitti/camera_color_right/image_raw | Type: sensor_msgs/msg/Image | Count: 77 | Serialization Format: cdr
                   Topic: /kitti/camera_gray_left/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 77 | Serialization Format: cdr
                   Topic: /kitti/camera_gray_left/image_raw | Type: sensor_msgs/msg/Image | Count: 77 | Serialization Format: cdr
                   Topic: /kitti/camera_gray_right/camera_info | Type: sensor_msgs/msg/CameraInfo | Count: 77 | Serialization Format: cdr
                   Topic: /kitti/camera_gray_right/image_raw | Type: sensor_msgs/msg/Image | Count: 77 | Serialization Format: cdr
                   Topic: /kitti/oxts/gps/fix | Type: sensor_msgs/msg/NavSatFix | Count: 77 | Serialization Format: cdr
                   Topic: /kitti/oxts/gps/vel | Type: geometry_msgs/msg/TwistStamped | Count: 77 | Serialization Format: cdr
                   Topic: /kitti/oxts/imu | Type: sensor_msgs/msg/Imu | Count: 77 | Serialization Format: cdr
                   Topic: /kitti/velo/pointcloud | Type: sensor_msgs/msg/PointCloud2 | Count: 77 | Serialization Format: cdr
                   Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: 77 | Serialization Format: cdr
                   Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: 77 | Serialization Format: cdr
Service:           0
Service information: 
```

## Example of running docker container in non-interactive mode

Inside the docker container, you can run the following commands to convert kitti sequences to rosbag2 mcap file. Note that `kitti2bag` is functionally equivalent to `ros2 run kitti2bag kitti2bag`.

## Convert KITTI sequences in non-interactive mode

Change the `$SEQUENCE` variable in `docker_convert.sh` script to convert the `0001` and `0002` sequences of `2011_09_26` KITTI dataset respectively in non-interactive mode. Then run the following:

```bash
./docker_convert_raw.sh
```

For `0001` sequence, the script basically runs the following command:

```bash
docker run -t --rm --tty \
    -e LOCAL_USER_ID=$(id -u) \
    -e LOCAL_GROUP_ID=$(id -g) \
    -v /path/to/kitti:/data \
    -v /path/to/output:/output \
    kitti2bag:ros2-jazzy \
    bash -c "kitti2bag raw_synced /data -t 2011_09_26 -r 0001 -o /output"
```

## Test the converted bag file

### Play the mcap file

Run docker container in interactive mode.

```bash
./docker_interact.sh
```

Inside the interactive container:

```bash
ros2 bag play /output/kitti_2011_09_26_drive_0002_synced
```

### Echo the published topics

Attach to the previously running container:

```bash
./docker_attach.sh
```

which basically runs the following:

```bash
docker exec -it kitti2bag-ros2-jazzy bash
```

Inside the attached docker container:

```bash
ros2 topic list
```
