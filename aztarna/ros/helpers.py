#!/usr/bin/env python
# -*- coding: utf-8 -*-

from aztarna.commons import BaseNode, BaseTopic, BaseService, BaseHost


class ROSHost(BaseHost):
    """
    Class for keeping all the attributes of a ROS Node.Extends:class:`aztarna.commons.BaseHost`
    """
    def __init__(self, address, port):
        super().__init__()
        self.address = address
        self.port = port
        self.communications = []
        self.services = []

    def __repr__(self):
        if len(self.nodes) == 0:
            return "Address: {}".format(self.address)
        return "Address: {}, Nodes: {}".format(self.address, self.nodes)



class Node(BaseNode):
    """
    Node class, an extension of the BaseNode
    """
    def __init__(self, name):
        super().__init__()
        self.name = name
        self.published_topics = []
        self.subscribed_topics = []
        self.services = []

    def __str__(self):
        return '{} XMLRPCUri: http://{}:{}'.format(self.name, self.address, self.port)


class Topic(BaseTopic):
    """
    Topic class, an extension of BaseTopic
    """
    def __init__(self, name, topic_type):
        super().__init__()
        self.name = name
        self.type = topic_type

    def __str__(self):
        return self.name + '(Type: ' + self.type + ')'


class Service(BaseService):
    """
    Service class, an extension of BaseService
    """
    def __init__(self, name):
        super().__init__()
        self.name = name

    def __str__(self):
        return '{}'.format(self.name)
