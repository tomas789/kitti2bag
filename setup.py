#!/usr/bin/env python

from setuptools import setup

setup(name='kitti2bag',
      version='1.4',
      description='Convert KITTI dataset to ROS bag file the easy way!',
      author='Tomas Krejci',
      author_email='tomas789@gmail.com',
      url='https://github.com/tomas789/kitti2bag/',
      download_url = 'https://github.com/tomas789/kitti2bag/archive/1.4.zip',
      keywords = ['dataset', 'ros', 'rosbag', 'kitti'],
      scripts=['bin/kitti2bag'],
      install_requires=['pykitti', 'progressbar2']
      )
