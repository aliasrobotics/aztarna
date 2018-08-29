#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup, find_packages

setup(
    name='aztarna',
    version='1.0',
    packages=['aztarna',
              'aztarna.ros',
              'aztarna.sros',
              ],
    url='https://www.aliasrobotics.com',
    project_urls={
        'Source Code': 'https://github.com/aliasrobotics/aztarna'
    },
    license='GPLv2',
    author='Alias Robotics',
    author_email='contact(at)aliasrobotics.com',
    description='A footprinting tool for ROS and SROS systems',
    keywords=['network', 'footprinting', 'ros', 'sros'],
    entry_points = {
        'console_scripts': ['aztarna=aztarna.cmd:main'],
    },
    install_requires=[
        'aiohttp==3.4.1',
        'aiohttp-xmlrpc==0.7.4',
        'async-timeout==3.0.0',
        'attrs==18.1.0',
        'chardet==3.0.4',
        'idna==2.7',
        'idna-ssl==1.1.0',
        'lxml==4.2.4',
        'multidict==4.3.1',
        'scapy==2.4.0',
        'yarl==1.2.6'
    ],
    include_package_data=True
)
