#!/usr/bin/env python
# -*- coding: utf-8 -*-
import asyncio
import ipaddress
import logging
from ipaddress import IPv4Address, ip_network

from aztarna.commons import *

logger = logging.getLogger(__name__)


class BaseHostROS(BaseRobotHost):
    """
    A base class for ROS hosts
    """
    def __init__(self):
        super().__init__()
        self.nodes = []


class BaseNodeROS:
    """
    A base class for ROS nodes
    """
    def __init__(self):
        self.name = ''
        self.address = ''
        self.port = ''


class BaseTopicROS:
    """
    A base class for ROS topics
    """
    def __init__(self):
        self.name = ''
        self.type = ''


class BaseServiceROS:
    """
    A base class for ROS services
    """
    def __init__(self):
        self.name = ''


class ParameterROS:
    """
    A class representing a ROS parameter
    """
    def __init__(self):
        self.name = ''
        self.type = ''
        self.value = ''


class CommunicationROS:
    """
    A class representing a ROS communication, including the publishers, subscribers and the topics involveds
    """
    def __init__(self, topic):
        self.publishers = []  # Node type
        self.subscribers = []  # Node type
        self.topic = topic # Topic() object
