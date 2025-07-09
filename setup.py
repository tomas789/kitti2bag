#!/usr/bin/env python3

from setuptools import setup, find_packages

setup(
    name='kitti2bag',
    version='2.0.0',
    description='Convert KITTI dataset to ROS2 bag file the easy way!',
    author='Tomas Krejci',
    author_email='tomas@krej.ci',
    url='https://github.com/tomas789/kitti2bag/',
    download_url = 'https://github.com/tomas789/kitti2bag/archive/2.0.0.zip',
    keywords = ['dataset', 'ros2', 'rosbag2', 'kitti'],
    packages=find_packages(),
    entry_points = {
        'console_scripts': ['kitti2bag=kitti2bag.__main__:main'],
    },
    install_requires=['pykitti', 'progressbar2'],
    zip_safe=False,
)
