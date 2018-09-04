#!/usr/bin/env python
# -*- coding: utf-8 -*-
import asyncio
import ipaddress
from ipaddress import IPv4Address, ip_network


class BaseScanner:

    def __init__(self, ports=[11311], extended=False):
        self.host_list = []
        self.ports = ports
        self.extended = extended
        self.input = False
        self._rate = 1000
        self.semaphore = asyncio.Semaphore(self._rate)

    @property
    def rate(self):
        return self._rate

    @rate.setter
    def rate(self, rate):
        self._rate = rate
        self.semaphore = asyncio.Semaphore(rate)



    def load_from_file(self, filename):
        with open(filename, 'r') as file:
            for line in file.readlines():
                address = ipaddress.ip_address(line.rstrip('\n'))
                self.host_list.append(address)

    def load_range(self, net_range):
        network = ip_network(net_range)
        if network.netmask == IPv4Address('255.255.255.255'):
            self.host_list = [IPv4Address(net_range)]
        else:
            self.host_list = list(network.hosts())

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





