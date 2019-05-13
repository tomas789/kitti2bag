import os
import shutil
from os.path import abspath, dirname, exists, join
from zipfile import ZipFile

import pytest
import requests
import rosbag
import tqdm
import yaml
from click.testing import CliRunner
from six import BytesIO

# Disable tqdm background monitor which does not exit cleanly with pytest in python 2
tqdm.monitor_interval = 0

from kitti2bag import cli

TESTS_DIR = dirname(abspath(__file__))
DATA_DIR = join(TESTS_DIR, 'data')

DATA_URL_BASE = 'https://s3.eu-central-1.amazonaws.com/avg-kitti'


def download_and_extract(filename, output_dir):
    url = DATA_URL_BASE + filename
    print('Downloading {}...'.format(url))
    response = requests.get(url)
    response.raise_for_status()
    zipfile = ZipFile(BytesIO(response.content))
    zipfile.extractall(output_dir)


@pytest.fixture(scope='session')
def raw_data():
    raw_data_dir = join(DATA_DIR, 'raw')
    params = {
        'dir': raw_data_dir,
        'date': '2011_09_26',
        'drive': 48,
        'drive_dir': join(raw_data_dir, '2011_09_26/2011_09_26_drive_0048_sync')
    }
    if exists(raw_data_dir):
        return params

    os.makedirs(raw_data_dir)
    download_and_extract('/raw_data/2011_09_26_drive_0048/2011_09_26_drive_0048_sync.zip', raw_data_dir)
    download_and_extract('/raw_data/2011_09_26_calib.zip', raw_data_dir)

    return params


@pytest.fixture(scope='session')
def odom_data(raw_data):
    odom_data_dir = join(DATA_DIR, 'odom')
    params = {
        'dir': odom_data_dir,
        'sequence': 4,
        'sequence_dir': join(odom_data_dir, 'sequences/04')
    }
    if exists(odom_data_dir):
        return params

    if not exists(DATA_DIR):
        os.makedirs(DATA_DIR)
    download_and_extract('/data_odometry_calib.zip', DATA_DIR)
    shutil.move(join(DATA_DIR, 'dataset'), odom_data_dir)

    # create dummy image data for odom
    for i in range(4):
        shutil.copytree(
            join(raw_data['drive_dir'], 'image_0{}/data').format(i),
            join(params['sequence_dir'], 'image_{}').format(i)
        )

    return params


def clean_bag_info(info):
    del info['path']
    del info['version']
    del info['size']
    for t in info['types']:
        del t['md5']
    return info


def test_raw_synced(raw_data, tmpdir):
    runner = CliRunner()
    result = runner.invoke(
        cli, [
            'raw',
            '--date', raw_data['date'],
            '--drive', raw_data['drive'],
            '--input-dir', raw_data['dir'],
            '--output-dir', str(tmpdir),
        ],
        catch_exceptions=False
    )
    print(result.stdout_bytes.decode('utf8'))
    assert result.exit_code == 0

    expected_bagfile = tmpdir.join('kitti_2011_09_26_drive_0048_synced.bag')
    assert expected_bagfile.exists()
    with rosbag.Bag(str(expected_bagfile), 'r') as bag:
        info = yaml.safe_load(bag._get_yaml_info())

    # remove irrelevant fields
    info = clean_bag_info(info)

    expected = {
        'compression': 'none',
        'duration': 2.151645,
        'end': 1317046453.072943,
        'indexed': True,
        'messages': 308,
        'start': 1317046450.921298,
        'topics': [
            {'frequency': 9.6728,
             'messages': 22,
             'topic': '/kitti/camera_color/left/camera_info',
             'type': 'sensor_msgs/CameraInfo'},
            {'frequency': 9.6728,
             'messages': 22,
             'topic': '/kitti/camera_color/left/image_rect_color',
             'type': 'sensor_msgs/Image'},
            {'frequency': 9.6725,
             'messages': 22,
             'topic': '/kitti/camera_color/right/camera_info',
             'type': 'sensor_msgs/CameraInfo'},
            {'frequency': 9.6725,
             'messages': 22,
             'topic': '/kitti/camera_color/right/image_rect_color',
             'type': 'sensor_msgs/Image'},
            {'frequency': 9.6725,
             'messages': 22,
             'topic': '/kitti/camera_gray/left/camera_info',
             'type': 'sensor_msgs/CameraInfo'},
            {'frequency': 9.6725,
             'messages': 22,
             'topic': '/kitti/camera_gray/left/image_rect',
             'type': 'sensor_msgs/Image'},
            {'frequency': 9.6709,
             'messages': 22,
             'topic': '/kitti/camera_gray/right/camera_info',
             'type': 'sensor_msgs/CameraInfo'},
            {'frequency': 9.6709,
             'messages': 22,
             'topic': '/kitti/camera_gray/right/image_rect',
             'type': 'sensor_msgs/Image'},
            {'frequency': 9.9951,
             'messages': 22,
             'topic': '/kitti/oxts/gps/fix',
             'type': 'sensor_msgs/NavSatFix'},
            {'frequency': 9.9951,
             'messages': 22,
             'topic': '/kitti/oxts/gps/vel',
             'type': 'geometry_msgs/TwistStamped'},
            {'frequency': 9.9951,
             'messages': 22,
             'topic': '/kitti/oxts/imu',
             'type': 'sensor_msgs/Imu'},
            {'frequency': 9.6715,
             'messages': 22,
             'topic': '/kitti/velo/pointcloud',
             'type': 'sensor_msgs/PointCloud2'},
            {'frequency': 9.9951,
             'messages': 22,
             'topic': '/tf',
             'type': 'tf2_msgs/TFMessage'},
            {'frequency': 9.9951,
             'messages': 22,
             'topic': '/tf_static',
             'type': 'tf2_msgs/TFMessage'}
        ],
        'types': [
            {'type': 'geometry_msgs/TwistStamped'},
            {'type': 'sensor_msgs/CameraInfo'},
            {'type': 'sensor_msgs/Image'},
            {'type': 'sensor_msgs/Imu'},
            {'type': 'sensor_msgs/NavSatFix'},
            {'type': 'sensor_msgs/PointCloud2'},
            {'type': 'tf2_msgs/TFMessage'}
        ]
    }

    assert info == expected

    tmpdir.remove()


@pytest.mark.parametrize('color', ['gray', 'color'])
def test_odom(odom_data, tmpdir, color):
    runner = CliRunner()
    result = runner.invoke(
        cli, [
            'odom',
            '--sequence', odom_data['sequence'],
            '--color', color,
            '--input-dir', odom_data['dir'],
            '--output-dir', str(tmpdir),
        ],
        catch_exceptions=False
    )
    print(result.stdout_bytes.decode('utf8'))
    assert result.exit_code == 0

    expected_bagfile = tmpdir.join('kitti_data_odometry_{}_sequence_04.bag'.format(color))
    assert expected_bagfile.exists()
    with rosbag.Bag(str(expected_bagfile), 'r') as bag:
        info = yaml.safe_load(bag._get_yaml_info())

    info = clean_bag_info(info)

    expected = {
        'compression': 'none',
        'duration': 2.187759,
        'end': 1317038402.187759,
        'indexed': True,
        'messages': 88,
        'start': 1317038400.0,
        'topics': [
            {'frequency': 9.6034,
             'messages': 22,
             'topic': '/kitti/camera_{}/left/camera_info'.format(color),
             'type': 'sensor_msgs/CameraInfo'},
            {'frequency': 9.6034,
             'messages': 22,
             'topic': '/kitti/camera_{}/left/image_rect{}'.format(
                 color, '' if color == 'gray' else '_color'),
             'type': 'sensor_msgs/Image'},
            {'frequency': 9.6034,
             'messages': 22,
             'topic': '/kitti/camera_{}/right/camera_info'.format(color),
             'type': 'sensor_msgs/CameraInfo'},
            {'frequency': 9.6034,
             'messages': 22,
             'topic': '/kitti/camera_{}/right/image_rect{}'.format(
                 color, '' if color == 'gray' else '_color'),
             'type': 'sensor_msgs/Image'}
        ],
        'types': [
            {'type': 'sensor_msgs/CameraInfo'},
            {'type': 'sensor_msgs/Image'}
        ]
    }

    assert info == expected

    tmpdir.remove()
