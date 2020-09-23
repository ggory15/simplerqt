#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['talos_wbc_gui'],
    package_dir={'': 'src'},
<<<<<<< HEAD
    requires=['roscpp'],
=======
    requires=['std_msgs', 'roscpp'],
>>>>>>> 2b56c27807e53af2e5e8d33d05a3a7d50dd24bf5
    scripts=['scripts/talos_wbc_gui']
)

setup(**d)