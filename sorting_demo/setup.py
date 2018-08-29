#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
"""
setup_args = generate_distutils_setup(
    package=['sorting_demo'],
    package_dir={'': 'src'},
)

setup(**setup_args)
"""

d = generate_distutils_setup()
d['packages'] = ['sorting_demo',
                 'sorting_demo.utils',
                 'sorting_demo.object_detection',
                 'intera_interface', 'intera_control',
                 'intera_dataflow',
                 'intera_io',
                 'intera_joint_trajectory_action',
                 'intera_motion_interface']
d['package_dir'] = {'': 'src', 'utils': 'src/utils'}

setup(**d, requires=['flask'])
