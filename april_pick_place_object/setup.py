#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['april_pick_place_object'],
 package_dir={'april_pick_place_object': 'src/april_pick_place_object'}
)

setup(**d)
