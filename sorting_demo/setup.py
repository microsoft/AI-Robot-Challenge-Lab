#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['intera_interface', 'intera_control', 'intera_dataflow',
                 'intera_io', 'intera_joint_trajectory_action', 'intera_motion_interface']
d['package_dir'] = {'': 'src'}

setup(**d)
