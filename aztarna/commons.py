#!/usr/bin/env python
# -*- coding: utf-8 -*-


class BaseScanner:

    def __init__(self, net_range='', ports=[11311], extended=False):
        self.net_range = net_range
        self.ports = ports
        self.extended = extended
        self.input = False

    def load_from_file(self,filename):
        raise NotImplementedError

    def scan(self):
        raise NotImplementedError

    def print_results(self):
        raise NotImplementedError

    def write_to_file(self, out_file):
        raise NotImplementedError


class BaseHost:
    def __init__(self):
        self.address = ''
        self.port = ''
        self.nodes = []


class BaseNode:
    def __init__(self):
        self.name = ''
        self.address = ''
        self.port = ''


class BaseTopic:
    def __init__(self):
        self.name = ''
        self.type = ''


class BaseService:
    def __init__(self):
        self.name = ''


class Parameter:
    def __init__(self):
        self.name = ''
        self.type = ''
        self.value = ''


class Communication:
    def __init__(self, topic):
        self.publishers = []  # Node type
        self.subscribers = []  # Node type
        self.topic = topic # Topic() object





