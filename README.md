# kitti2bag

Convert [KITTI](http://www.cvlibs.net/datasets/kitti/index.php) dataset to ROS bag file the easy way!

![KITTI playback preview](https://tomas789.github.io/kitti2bag/img/kitti_playback.png)

## TODOs

Help me make this feature rich and complete. Just fork this repo, implement new features (very easy in this case) and make [pull request](https://github.com/tomas789/kitti2bag/pulls).

Feature request list:
 * export velodyne laser data
 * export GPS
 * make [URDF](http://wiki.ros.org/urdf) of a car so transformations between frames are easily done by ROS itself.
 * export odometry
 * deal with tracklets
 * support for unsynced+unrectified version

## How to install it?

It is very easy! Just run
```bash
pip install kitti2bag
```

## How to run it?

One example if better then thousand words so here it is

```bash
$ wget http://kitti.is.tue.mpg.de/kitti/raw_data/2011_09_26_drive_0002/2011_09_26_drive_0002_sync.zip
$ wget http://kitti.is.tue.mpg.de/kitti/raw_data/2011_09_26_calib.zip
$ unzip unzip 2011_09_26_drive_0002_sync.zip
$ unzip 2011_09_26_calib.zip
$ kitti2bag 2011_09_26 0002
Loading OXTS data from 2011_09_26_drive_0002_sync...
Found 77 OXTS measurements...
done.
Loading OXTS timestamps from 2011_09_26_drive_0002_sync...
Found 77 timestamps...
done.
BAG: kitti_2011_09_26_drive_0002_sync.bag
```

That's it. You have file `kitti_2011_09_26_drive_0002_sync.bag` that contains your data. You can check it.
```bash
$ rosbag info kitti_2011_09_26_drive_0002_sync.bag
path:        kitti_2011_09_26_drive_0002_sync.bag
version:     2.0
duration:    3.2s
start:       Nov 28 2016 12:34:24.50 (1480332864.50)
end:         Nov 28 2016 12:34:27.70 (1480332867.70)
size:        34.3 MB
messages:    154
compression: none [39/39 chunks]
types:       sensor_msgs/Image [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/Imu   [6a62c6daae103f4ff57a132d6f95cec2]
topics:      /kitti/camera/gray/left   77 msgs    : sensor_msgs/Image
             /kitti/oxts/imu           77 msgs    : sensor_msgs/Imu
```

Other source files can be found at [KITTI raw data](http://www.cvlibs.net/datasets/kitti/raw_data.php) page.

## Bug reporting, support and feature requests.

I appreciate [pull requests](https://github.com/tomas789/kitti2bag/pulls) with bug fixes and new features. You you want to help with something please use [GitHub issue tracker](https://github.com/tomas789/kitti2bag/issues).

## Related works

 * [pykitti](https://github.com/utiasSTARS/pykitti) is very simple library for dealing with KITTI dataset in python. 
 * [kitti_player](https://github.com/tomas789/kitti_player) allows to play dataset directly. No bag file needed. I found difficult to get it work. Some bug fixed can be found in [my fork of kitti_player](https://github.com/tomas789/kitti_player) but still not good enough.
