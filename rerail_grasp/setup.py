#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup( packages=['rerail_grasp'], package_dir={'': 'src'}, scripts=['scripts/hello_robot_grasp'])

setup(**setup_args)