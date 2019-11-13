#!/usr/bin/env python

from setuptools import setup

setup(
    name='kitti2bag',
    version='1.5',
    description='Convert KITTI dataset to ROS bag file the easy way!',
    author='Tomas Krejci',
    author_email='tomas@krej.ci',
    url='https://github.com/tomas789/kitti2bag/',
    download_url = 'https://github.com/tomas789/kitti2bag/archive/1.5.zip',
    keywords = ['dataset', 'ros', 'rosbag', 'kitti'],
    entry_points = {
        'console_scripts': ['kitti2bag=kitti2bag.__main__:main'],
    },
    install_requires=[
        'catkin_pkg',
        'progressbar2',
        'pykitti',
        'pyyaml',
        'rospkg',
    ]
)
