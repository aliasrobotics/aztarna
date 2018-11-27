#!/usr/bin/env python
# -*- coding: utf-8 -*-
import asyncio
import ipaddress
import logging
from ipaddress import IPv4Address, ip_network

logger = logging.getLogger(__name__)


class RobotAdapter:
    """
    BaseScanner class, an abstraction for different type scans
    """
    def __init__(self, ports=[80], extended=False):
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
        """
        Load a range of ipv4 addresses to scan and add them The :class:BaseScanner host_list attribute
        :param filename: name of the input file
        """
        with open(filename, 'r') as file:
            for line in file.readlines():
                try:
                    address = ipaddress.ip_address(line.rstrip('\n'))
                    self.host_list.append(address)
                except ValueError:
                    logger.warning('Invalid IP address in input file')

    def load_range(self, net_range):
        """
        Transform ipv4 address strings to pythons ipaddress library type objects for scanning purposes
        :param net_range: A range of string type IPv4 addresses
        """
        network = ip_network(net_range)
        if network.netmask == IPv4Address('255.255.255.255'):
            self.host_list = [IPv4Address(net_range)]
        else:
            self.host_list = list(network.hosts())

    @staticmethod
    async def stream_as_generator(loop, stream):
        reader = asyncio.StreamReader(loop=loop)
        reader_protocol = asyncio.StreamReaderProtocol(reader)
        await loop.connect_read_pipe(lambda: reader_protocol, stream)

        while True:
            line = await reader.readline()
            if not line:  # EOF.
                break
            yield line

    def scan(self):
        raise NotImplementedError

    def scan_pipe_main(self):
        raise NotImplementedError

    def print_results(self):
        raise NotImplementedError

    def write_to_file(self, out_file):
        raise NotImplementedError


class BaseRobotHost:
    """
    A base class for different type of Robot hosts
    """
    def __init__(self):
        self.address = ''
        self.port = ''
