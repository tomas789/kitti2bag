# kitti2bag

[![Gitter](https://badges.gitter.im/kitti2bag/community.svg)](https://gitter.im/kitti2bag/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge) [![Build Status](https://travis-ci.org/tomas789/kitti2bag.svg?branch=master)](https://travis-ci.org/tomas789/kitti2bag)

Convert [KITTI](http://www.cvlibs.net/datasets/kitti/index.php) dataset to ROS bag file the easy way!

![KITTI playback preview](https://raw.githubusercontent.com/tomas789/kitti2bag/gh-pages/img/kitti_playback.png)
![KITTI rviz preview](https://raw.githubusercontent.com/AlterLimbo/kitti2bag/gh-pages/img/kitti_rviz.png)

## Collaboration

This package enjoyed significant interest from more people then I could have thought at the beginning. I'm really glad to see that. I see many PRs and issues being raised but my day job does not allow me to push this repository further. In order to allow this package to prosper, I'm opening it up for the community. I'm more than happy to add you as a collaborator to this repository. Just send me an email. 

And by the way. To ensure we maintain the quality of the repo you are required to get the PR approval from at least one other collaborator. You can use the [Gitter](https://gitter.im/kitti2bag/community?utm_source=share-link&utm_medium=link&utm_campaign=share-link) to communicate with others.

## TODOs

Help me make this feature rich and complete. Just fork this repo, implement new features (very easy in this case) and make [pull request](https://github.com/tomas789/kitti2bag/pulls).

Feature request list:
 * make [URDF](http://wiki.ros.org/urdf) of a car so transformations between frames are easily done by ROS itself.
 * deal with tracklets
 * support for unsynced+unrectified version
 * provide documentation via [ROS wiki](wiki.ros.org)
 * provide simple GUI
 * distribute publically available bagfiles (is there a reliable public storage for this purpose?)
 * export only subset of sensors

## Contributions

Thanks to the work of @jnitsch, _kitti2bag_ can now export velodyne laser data and dynamic _tf_ transformations. Thanks to @emreay-, this tool can now convert odometry datasets too. Thank you both!

## How to install it?

It is very easy! On the machine with ROS installed, just run
install pykitti from source
```bash
git clone https://github.com/utiasSTARS/pykitti.git
cd pykitti
sudo python setup.py install
```
install kitti2bag
```bash
cd kitti2bag
sudo python setup.py install
```

## How to run it?

One example is better then thousand words so here it is

```bash
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0002_sync.zip
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip
$ unzip 2011_09_26_drive_0002_sync.zip
$ unzip 2011_09_26_calib.zip
$ kitti2bag -t 2011_09_26 -r 0002 raw_sync .
Exporting static transformations
Exporting time dependent transformations
Exporting IMU
Exporting camera 0
100% (77 of 77) |##########################| Elapsed Time: 0:00:00 Time: 0:00:00
Exporting camera 1
100% (77 of 77) |##########################| Elapsed Time: 0:00:00 Time: 0:00:00
Exporting camera 2
100% (77 of 77) |##########################| Elapsed Time: 0:00:01 Time: 0:00:01
Exporting camera 3
100% (77 of 77) |##########################| Elapsed Time: 0:00:01 Time: 0:00:01
Exporting velodyne data
100% (77 of 77) |##########################| Elapsed Time: 0:00:15 Time: 0:00:15
## OVERVIEW ##
path:        kitti_2011_09_26_drive_0002_sync.bag
version:     2.0
duration:    7.8s
start:       Sep 26 2011 13:02:44.33 (1317042164.33)
end:         Sep 26 2011 13:02:52.16 (1317042172.16)
size:        417.2 MB
messages:    1078
compression: none [308/308 chunks]
types:       geometry_msgs/TwistStamped [98d34b0043a2093cf9d9345ab6eef12e]
             sensor_msgs/CameraInfo     [c9a58c1b0b154e0e6da7578cb991d214]
             sensor_msgs/Image          [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/Imu            [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/NavSatFix      [2d3a8cd499b9b4a0249fb98fd05cfa48]
             sensor_msgs/PointCloud2    [1158d486dd51d683ce2f1be655c3c181]
             tf2_msgs/TFMessage         [94810edda583a504dfda3829e70d7eec]
topics:      /kitti/camera_color_left/camera_info    77 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_color_left/image          77 msgs    : sensor_msgs/Image         
             /kitti/camera_color_right/camera_info   77 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_color_right/image         77 msgs    : sensor_msgs/Image         
             /kitti/camera_gray_left/camera_info     77 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_gray_left/image           77 msgs    : sensor_msgs/Image         
             /kitti/camera_gray_right/camera_info    77 msgs    : sensor_msgs/CameraInfo    
             /kitti/camera_gray_right/image          77 msgs    : sensor_msgs/Image         
             /kitti/oxts/gps/fix                     77 msgs    : sensor_msgs/NavSatFix     
             /kitti/oxts/gps/vel                     77 msgs    : geometry_msgs/TwistStamped
             /kitti/oxts/imu                         77 msgs    : sensor_msgs/Imu           
             /kitti/velo/pointcloud                  77 msgs    : sensor_msgs/PointCloud2   
             /tf                                     77 msgs    : tf2_msgs/TFMessage        
             /tf_static                               1 msgs    : tf2_msgs/TFMessage
```


That's it. You have file `kitti_2011_09_26_drive_0002_sync.bag` that contains your data.

Other raw files can be found at [KITTI raw data](http://www.cvlibs.net/datasets/kitti/raw_data.php) page.

For odometry data from [KITTI odometry](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) page.

```bash
$ unzip data_odometry_color.zip
$ unzip data_odometry_gray.zip
$ unzip data_odometry_poses.zip
$ unzip data_odometry_velodyne.zip
$ unzip data_odometry_calib.zip
$ kitti2bag -s 04 odom ./dataset/
Odometry dataset sequence 04 has ground truth information (poses).
Exporting static transformations
Exporting time dependent transformations
Exporting camera 0
100% (271 of 271) |##########################| Elapsed Time: 0:00:01 Time:  0:00:01
Exporting camera 1
100% (271 of 271) |##########################| Elapsed Time: 0:00:01 Time:  0:00:01
Exporting camera 2
100% (271 of 271) |##########################| Elapsed Time: 0:00:02 Time:  0:00:02
Exporting camera 3
100% (271 of 271) |##########################| Elapsed Time: 0:00:02 Time:  0:00:02
Exporting velodyne data
100% (271 of 271) |##########################| Elapsed Time: 0:00:30 Time:  0:00:30
## OVERVIEW ##
path:        kitti_odometry_sequence_04.bag
version:     2.0
duration:    28.1s
start:       Jul 26 2019 15:12:12.70 (1564125132.70)
end:         Jul 26 2019 15:12:40.81 (1564125160.81)
size:        1.4 GB
messages:    2711
compression: none [1084/1084 chunks]
types:       sensor_msgs/CameraInfo  [c9a58c1b0b154e0e6da7578cb991d214]
             sensor_msgs/Image       [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/PointCloud2 [1158d486dd51d683ce2f1be655c3c181]
             tf2_msgs/TFMessage      [94810edda583a504dfda3829e70d7eec]
topics:      /kitti/camera_color_left/camera_info    271 msgs    : sensor_msgs/CameraInfo 
             /kitti/camera_color_left/image          271 msgs    : sensor_msgs/Image      
             /kitti/camera_color_right/camera_info   271 msgs    : sensor_msgs/CameraInfo 
             /kitti/camera_color_right/image         271 msgs    : sensor_msgs/Image      
             /kitti/camera_gray_left/camera_info     271 msgs    : sensor_msgs/CameraInfo 
             /kitti/camera_gray_left/image           271 msgs    : sensor_msgs/Image      
             /kitti/camera_gray_right/camera_info    271 msgs    : sensor_msgs/CameraInfo 
             /kitti/camera_gray_right/image          271 msgs    : sensor_msgs/Image      
             /kitti/velo/pointcloud                  271 msgs    : sensor_msgs/PointCloud2
             /tf                                     271 msgs    : tf2_msgs/TFMessage     
             /tf_static                                1 msg     : tf2_msgs/TFMessage

```

If you got an error saying something like _command not found_ it means that your python installation is in bad shape. You might try running 
```python -m kitti2bag -t 2011_09_26 -r 0002 raw_sync .```
Or maybe use Docker.

### Prefer Docker?

That is easy too. There is a pre-built image `tomas789/kitti2bag`. 

```bash
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0002_sync.zip
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip
$ unzip 2011_09_26_drive_0002_sync.zip
$ unzip 2011_09_26_calib.zip
$ docker run -v `pwd`:/data -it tomas789/kitti2bag -t 2011_09_26 -r 0002 raw_sync
Exporting static transformations
Exporting time dependent transformations
...
```

This might also be a better alternative if you are having troubles installing the package. 

## Bug reporting, support and feature requests.

I appreciate [pull requests](https://github.com/tomas789/kitti2bag/pulls) with bug fixes and new features. You you want to help with something please use [GitHub issue tracker](https://github.com/tomas789/kitti2bag/issues).

## Related works

 * [pykitti](https://github.com/utiasSTARS/pykitti) is very simple library for dealing with KITTI dataset in python. 
 * [kitti_player](https://github.com/tomas789/kitti_player) allows to play dataset directly. No bag file needed. I found difficult to get it work. Some bug fixed can be found in [my fork of kitti_player](https://github.com/tomas789/kitti_player) but still not good enough.
