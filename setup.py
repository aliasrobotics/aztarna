#!/usr/bin/env python
# -*- coding: utf-8 -*-

from distutils.core import setup

setup(
    name='aztarna',
    version='1.0',
    packages=[
        'aztarna',
        'aztarna.ros',
        'aztarna.sros',
    ],
    url='https://www.aliasrobotics.com',
    project_urls={
        'Source Code': 'https://github.com/aliasrobotics/aztarna'
    },
    license='GPLv2',
    author='Alias Robotics',
    author_email='contact@aliasrobotics.com',
    description='A footprinting tool for ROS and SROS systems',
    keywords=['network', 'footprinting', 'ros', 'sros'],
    scripts=['bin/aztarna']
)
