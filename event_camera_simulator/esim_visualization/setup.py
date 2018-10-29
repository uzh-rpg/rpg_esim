#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['esim_visualization'],
    package_dir={'': 'py'},
    install_requires=['rospy', 'sensor_msgs', 'esim_msgs'],
    )

setup(**d)
