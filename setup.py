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
                        "aiohttp==3.4.2",
                        "aiohttp-xmlrpc==0.7.4",
                        "alabaster==0.7.11",
                        "argcomplete==1.9.4",
                        "asn1crypto==0.24.0",
                        "async-timeout==3.0.0",
                        "attrs==18.2.0",
                        "Babel==2.6.0",
                        "certifi==2018.8.24",
                        "cffi==1.11.5",
                        "chardet==3.0.4",
                        "cryptography==2.3.1",
                        "docutils==0.14",
                        "idna==2.7",
                        "idna-ssl==1.1.0",
                        "imagesize==1.1.0",
                        "Jinja2==2.10",
                        "lxml==4.2.4",
                        "MarkupSafe==1.0",
                        "multidict==4.3.1",
                        "packaging==17.1",
                        "pycparser==2.18",
                        "Pygments==2.2.0",
                        "pyparsing==2.2.0",
                        "pytz==2018.5",
                        "requests==2.19.1",
                        "scapy==2.4.0",
                        "six==1.11.0",
                        "snowballstemmer==1.2.1",
                        "Sphinx==1.7.9",
                        "sphinxcontrib-websupport==1.1.0",
                        "urllib3==1.23",
                        "uvloop==0.11.2",
                        "yarl==1.2.6"
    ],
    include_package_data=True
)
