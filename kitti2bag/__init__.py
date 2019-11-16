#!env python
# -*- coding: utf-8 -*-
from __future__ import division, print_function

import os
import sys
from collections import namedtuple
from datetime import datetime

import click
import cv2
import numpy as np
import pykitti
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
from geometry_msgs.msg import Transform, TransformStamped, TwistStamped
from sensor_msgs.msg import CameraInfo, Imu, NavSatFix, PointField
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, quaternion_from_matrix
from tf2_msgs.msg import TFMessage
from tqdm import tqdm

CameraDetails = namedtuple('CameraDetails', ['nr', 'frame_id', 'topic_id', 'is_rgb'])
cameras = [
    CameraDetails(0, 'camera_gray_left', '/kitti/camera_gray/left', False),
    CameraDetails(1, 'camera_gray_right', '/kitti/camera_gray/right', False),
    CameraDetails(2, 'camera_color_left', '/kitti/camera_color/left', True),
    CameraDetails(3, 'camera_color_right', '/kitti/camera_color/right', True)
]
# All of the cameras share the same camera #0 frame after rectification,
# which is the case for both the raw synced and odometry datasets.
rectified_camera_frame_id = cameras[0].frame_id


def to_rostime(dt):
    """Convert datetime from python format to ROS format."""
    tsecs = (dt - datetime.utcfromtimestamp(0)).total_seconds()
    return rospy.Time.from_sec(tsecs)


def read_timestamps(directory):
    with open(os.path.join(directory, 'timestamps.txt')) as f:
        timestamps = []
        for line in f:
            if len(line) == 1:
                continue
            dt = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
            timestamps.append(dt)
    return timestamps


def inv(transform):
    """Invert rigid body transformation matrix"""
    R = transform[0:3, 0:3]
    t = transform[0:3, 3]
    t_inv = -1 * R.T.dot(t)
    transform_inv = np.eye(4)
    transform_inv[0:3, 0:3] = R.T
    transform_inv[0:3, 3] = t_inv
    return transform_inv


def save_imu_data(bag, kitti, imu_frame_id, topic):
    print("Exporting IMU")
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        q = quaternion_from_euler(oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw)
        imu = Imu()
        imu.header.frame_id = imu_frame_id
        imu.header.stamp = to_rostime(timestamp)
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = oxts.packet.af
        imu.linear_acceleration.y = oxts.packet.al
        imu.linear_acceleration.z = oxts.packet.au
        imu.angular_velocity.x = oxts.packet.wf
        imu.angular_velocity.y = oxts.packet.wl
        imu.angular_velocity.z = oxts.packet.wu
        bag.write(topic, imu, t=imu.header.stamp)


def save_dynamic_tf(bag, timestamps, tf_matrices, child_frame_id):
    print("Exporting time dependent transformations")
    for timestamp, tf_matrix in zip(timestamps, tf_matrices):
        tf_msg = TFMessage()
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = to_rostime(timestamp)
        tf_stamped.header.frame_id = 'world'
        tf_stamped.child_frame_id = child_frame_id

        t = tf_matrix[0:3, 3]
        q = quaternion_from_matrix(tf_matrix)
        transform = Transform()

        transform.translation.x = t[0]
        transform.translation.y = t[1]
        transform.translation.z = t[2]

        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]

        tf_stamped.transform = transform
        tf_msg.transforms.append(tf_stamped)

        bag.write('/tf', tf_msg, tf_msg.transforms[0].header.stamp)


def save_camera_data(bag, kitti, camera, timestamps):
    print("Exporting camera {}".format(camera.nr))

    camera_info = CameraInfo()
    camera_info.header.frame_id = rectified_camera_frame_id
    camera_info.K = list(getattr(kitti.calib, 'K_cam{}'.format(camera.nr)).flat)
    camera_info.P = list(getattr(kitti.calib, 'P_rect_{}0'.format(camera.nr)).flat)
    # We do not include the D and R parameters from the calibration data since the images are
    # already undistorted and rectified to the camera #0 frame.
    camera_info.R = list(np.eye(3).flat)

    cv_bridge = CvBridge()

    image_paths = getattr(kitti, 'cam{}_files'.format(camera.nr))
    for timestamp, image_path in tqdm(list(zip(timestamps, image_paths))):
        cv_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        camera_info.height, camera_info.width = cv_image.shape[:2]
        encoding = 'bgr8' if camera.is_rgb else 'mono8'
        image_message = cv_bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        image_message.header.frame_id = rectified_camera_frame_id
        t = to_rostime(timestamp)
        image_message.header.stamp = t
        camera_info.header.stamp = t
        # Follow the naming conventions from
        # http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html
        image_topic_ext = '/image_rect_color' if camera.is_rgb else '/image_rect'
        bag.write(camera.topic_id + image_topic_ext, image_message, t=t)
        bag.write(camera.topic_id + '/camera_info', camera_info, t=t)


def save_velo_data(bag, kitti, velo_frame_id, topic):
    print("Exporting velodyne data")
    velo_path = os.path.join(kitti.data_path, 'velodyne_points')
    velo_datetimes = read_timestamps(velo_path)

    for dt, velo_filename in tqdm(list(zip(velo_datetimes, kitti.velo_files))):
        if dt is None:
            continue

        # read binary data
        scan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)

        # create header
        header = Header()
        header.frame_id = velo_frame_id
        header.stamp = to_rostime(dt)

        # fill PCL msg
        # uses the PCL PointXYZI struct's field names
        # http://docs.pointclouds.org/1.7.0/point__types_8hpp_source.html#l00380
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('intensity', 12, PointField.FLOAT32, 1)]
        pcl_msg = pcl2.create_cloud(header, fields, scan)

        bag.write(topic + '/pointcloud', pcl_msg, t=pcl_msg.header.stamp)


def get_static_transform(from_frame_id, to_frame_id, transform):
    t = transform[0:3, 3]
    q = quaternion_from_matrix(transform)
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = from_frame_id
    tf_msg.child_frame_id = to_frame_id
    tf_msg.transform.translation.x = float(t[0])
    tf_msg.transform.translation.y = float(t[1])
    tf_msg.transform.translation.z = float(t[2])
    tf_msg.transform.rotation.x = float(q[0])
    tf_msg.transform.rotation.y = float(q[1])
    tf_msg.transform.rotation.z = float(q[2])
    tf_msg.transform.rotation.w = float(q[3])
    return tf_msg


def save_static_transforms(bag, kitti, imu_frame_id, velo_frame_id):
    print("Exporting static transformations")

    T_base_link_to_imu = np.eye(4, 4)
    T_base_link_to_imu[0:3, 3] = [-2.71 / 2.0 - 0.05, 0.32, 0.93]

    # tf_static
    transforms = [
        (imu_frame_id, 'base_link', inv(T_base_link_to_imu)),
        (imu_frame_id, velo_frame_id, inv(kitti.calib.T_velo_imu)),
        (imu_frame_id, cameras[0].frame_id, inv(kitti.calib.T_cam0_imu)),
        (imu_frame_id, cameras[1].frame_id, inv(kitti.calib.T_cam1_imu)),
        (imu_frame_id, cameras[2].frame_id, inv(kitti.calib.T_cam2_imu)),
        (imu_frame_id, cameras[3].frame_id, inv(kitti.calib.T_cam3_imu))
    ]

    tfm = TFMessage()
    for transform in transforms:
        t = get_static_transform(from_frame_id=transform[0], to_frame_id=transform[1], transform=transform[2])
        tfm.transforms.append(t)
    for timestamp in kitti.timestamps:
        time = to_rostime(timestamp)
        for transform in tfm.transforms:
            transform.header.stamp = time
        bag.write('/tf_static', tfm, t=time)


def save_gps_fix_data(bag, kitti, gps_frame_id, topic):
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.frame_id = gps_frame_id
        navsatfix_msg.header.stamp = to_rostime(timestamp)
        navsatfix_msg.latitude = oxts.packet.lat
        navsatfix_msg.longitude = oxts.packet.lon
        navsatfix_msg.altitude = oxts.packet.alt
        navsatfix_msg.status.service = 1
        bag.write(topic, navsatfix_msg, t=navsatfix_msg.header.stamp)


def save_gps_vel_data(bag, kitti, gps_frame_id, topic):
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = gps_frame_id
        twist_msg.header.stamp = to_rostime(timestamp)
        twist_msg.twist.linear.x = oxts.packet.vf
        twist_msg.twist.linear.y = oxts.packet.vl
        twist_msg.twist.linear.z = oxts.packet.vu
        twist_msg.twist.angular.x = oxts.packet.wf
        twist_msg.twist.angular.y = oxts.packet.wl
        twist_msg.twist.angular.z = oxts.packet.wu
        bag.write(topic, twist_msg, t=twist_msg.header.stamp)


compression_choices = [rosbag.Compression.NONE, rosbag.Compression.BZ2, rosbag.Compression.LZ4]


@click.group()
@click.help_option('-h', '--help')
def cli():
    """Convert KITTI dataset to ROS bag file the easy way!"""
    pass


def common_options(f):
    f = click.option("-i", "--input-dir", required=False, default='.', show_default=True,
                     type=click.Path(exists=True, dir_okay=True, file_okay=False),
                     help="base directory of the dataset")(f)
    f = click.option("-o", "--output-dir", required=False, default='.', show_default=True,
                     type=click.Path(exists=True, dir_okay=True, file_okay=False),
                     help="output directory for the created bag file")(f)
    f = click.option("--compression", required=False,
                     type=click.Choice(compression_choices),
                     default=rosbag.Compression.NONE, show_default=True,
                     help="which compression to use for the created bag")(f)
    f = click.help_option('-h', '--help')(f)
    return f


@cli.command('raw')
@click.option("-t", "--date", required=True, metavar='DATE',
              help="date of the raw dataset (i.e. 2011_09_26)")
@click.option("-r", "--drive", required=True, type=int,
              help="drive number of the raw dataset (i.e. 1)")
@common_options
def convert_raw(date, drive, input_dir='.', output_dir='.', compression=rosbag.Compression.NONE):
    """Convert a raw synced+rectified KITTI dataset to a bag"""
    drive = str(drive).zfill(4)
    kitti = pykitti.raw(input_dir, date, drive)

    if len(kitti.timestamps) == 0:
        print('Dataset is empty? Exiting.', file=sys.stderr)
        sys.exit(1)

    # IMU
    imu_frame_id = 'imu_link'
    imu_topic = '/kitti/oxts/imu'
    gps_fix_topic = '/kitti/oxts/gps/fix'
    gps_vel_topic = '/kitti/oxts/gps/vel'
    velo_frame_id = 'velo_link'
    velo_topic = '/kitti/velo'

    # Export
    bag_name = "kitti_{}_drive_{}_synced.bag".format(date, drive)
    with rosbag.Bag(os.path.join(output_dir, bag_name), 'w', compression=compression) as bag:
        save_static_transforms(bag, kitti, imu_frame_id, velo_frame_id)
        imu_tf_matrices = [oxt.T_w_imu for oxt in kitti.oxts]
        save_dynamic_tf(bag, kitti.timestamps, imu_tf_matrices, imu_frame_id)
        save_imu_data(bag, kitti, imu_frame_id, imu_topic)
        save_gps_fix_data(bag, kitti, imu_frame_id, gps_fix_topic)
        save_gps_vel_data(bag, kitti, imu_frame_id, gps_vel_topic)
        for camera in cameras:
            camera_dir = os.path.join(kitti.data_path, 'image_{0:02d}'.format(camera.nr))
            timestamps = read_timestamps(camera_dir)
            save_camera_data(bag, kitti, camera, timestamps)
        save_velo_data(bag, kitti, velo_frame_id, velo_topic)

        print("## OVERVIEW ##")
        print(bag)


@cli.command('odom')
@click.option("-s", "--sequence", type=click.IntRange(0, 21), metavar='0...21', required=True,
              help="sequence number")
@click.option("-c", "--color", type=click.Choice(['gray', 'color', 'all']), required=True,
              help="which camera images to include in the bag")
@common_options
def convert_odom(sequence, color, input_dir='.', output_dir='.', compression=rosbag.Compression.NONE):
    """Convert an odometry KITTI dataset to a bag"""
    sequence_str = str(sequence).zfill(2)
    kitti = pykitti.odometry(input_dir, sequence_str)

    if len(kitti.timestamps) == 0:
        print('Dataset is empty? Exiting.', file=sys.stderr)
        sys.exit(1)

    # The odom timestamps are relative, add an arbitrary datetime to make them absolute
    base_timestamp = datetime(2011, 9, 26, 12, 0, 0)
    timestamps = [base_timestamp + timestamp for timestamp in kitti.timestamps]

    if 0 <= sequence < 10:
        print("Odometry dataset sequence {} has ground truth information (poses).".format(sequence_str))

    if color == 'color':
        camera_nrs = (2, 3)
    elif color == 'gray':
        camera_nrs = (0, 1)
    else:
        camera_nrs = list(range(4))

    # Export
    bag_name = "kitti_data_odometry_{}_sequence_{}.bag".format(color, sequence_str)
    with rosbag.Bag(os.path.join(output_dir, bag_name), 'w', compression=compression) as bag:
        save_dynamic_tf(bag, timestamps, kitti.poses, rectified_camera_frame_id)
        for camera_nr in camera_nrs:
            save_camera_data(bag, kitti, cameras[camera_nr], timestamps)
        print("## OVERVIEW ##")
        print(bag)


if __name__ == '__main__':
    cli()
