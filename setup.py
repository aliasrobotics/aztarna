#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

from setuptools import setup

setup(
    name='aztarna',
    version='1.2.1',
    packages=[
                'aztarna',
                'aztarna.ros',
                'aztarna.ros.ros',
                'aztarna.ros.sros',
                'aztarna.ros.industrial',
                'aztarna.ros.ros2',
                'aztarna.industrialrouters',
              ],
    url='https://www.aliasrobotics.com/research/aztarna.htm',
    project_urls={
        'Source Code': 'https://github.com/aliasrobotics/aztarna'
    },
    license='GPLv3',
    author='Alias Robotics',
    author_email='contact@aliasrobotics.com',
    description='A footprinting tool for ROS and SROS systems',
    long_description='''Aztarna, a footprinting tool for robots. 
    Provides researchers a way for researching internet connected ROS, SROS robots, as well as industrial routers.
    
    Alias Robotics supports original robot manufacturers assessing their security and improving their quality of software.
    By no means we encourage or promote the unauthorized tampering with running robotic systems.
    This can cause serious human harm and material damages.
    ''',
    keywords=['network', 'footprinting', 'ros', 'sros', 'ics', 'industrialrouters'],
    entry_points = {
        'console_scripts': ['aztarna=aztarna.cmd:main'],
    },
    install_requires=[
        'aiohttp==3.5.4',
        'aiohttp-xmlrpc==0.7.4',
        'alabaster==0.7.12',
        'argcomplete==1.9.5',
        'asn1crypto==0.24.0',
        'async-timeout==3.0.1',
        'attrs==19.1.0',
        'Babel==2.6.0',
        'certifi==2019.3.9',
        'cffi==1.12.2',
        'chardet==3.0.4',
        'Click==7.0',
        'click-plugins==1.1.1',
        'colorama==0.4.1',
        'cryptography==2.6.1',
        'dnspython==1.16.0',
        'docutils==0.14',
        'idna==2.8',
        'idna-ssl==1.1.0',
        'imagesize==1.1.0',
        'ipwhois==1.1.0',
        'Jinja2==2.10.1',
        'lxml==4.3.3',
        'MarkupSafe==1.1.1',
        'multidict==4.5.2',
        'packaging==19.0',
        'property==2.2',
        'pycparser==2.19',
        'Pygments==2.3.1',
        'pyparsing==2.4.0',
        'pytz==2019.1',
        'requests==2.22.0',
        'scapy==2.4.2',
        'shodan==1.12.1',
        'six==1.12.0',
        'snowballstemmer==1.2.1',
        'Sphinx==2.0.1',
        'sphinxcontrib-applehelp==1.0.1',
        'sphinxcontrib-devhelp==1.0.1',
        'sphinxcontrib-htmlhelp==1.0.2',
        'sphinxcontrib-jsmath==1.0.1',
        'sphinxcontrib-qthelp==1.0.2',
        'sphinxcontrib-serializinghtml==1.1.3',
        'sphinxcontrib-websupport==1.1.0',
        'urllib3>=1.24.2',
        'uvloop==0.12.2',
        'XlsxWriter==1.1.6',
        'yarl==1.3.0',
    ],
    include_package_data=True
)
