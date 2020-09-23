#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['talos_wbc_gui'],
    package_dir={'': 'src'},
    requires=['roscpp'],
    scripts=['scripts/talos_wbc_gui']
)

setup(**d)