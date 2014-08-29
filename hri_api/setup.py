#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['hri_api', 'hri_api.actions', 'hri_api.entities', 'hri_api.math', 'hri_api.query', 'hri_api.util'],
    package_dir={'': 'src/'},
)

setup(**d)
