#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tf2_sensor_msgs'],
    package_dir={'': 'src'},
    requires={'rospy','sensor_msgs','tf2_ros','orocos_kdl'}
)

setup(**d)

