#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SROS Scanner module.

:author: Gorka Olalde Mendia(@olaldiko), Xabier Perez Baskaran(@xabierpb)
"""

import asyncio
import logging
import random
import traceback
from ipaddress import AddressValueError

from aztarna.commons import RobotAdapter
from .helpers import SROSHost, get_node_info, get_policies, get_sros_certificate, find_node_ports

logger = logging.getLogger(__name__)

class SROSScanner(RobotAdapter):
    """
    SROS Scanner class, extending :class:`aztarna.commons.BaseScanner`.
    """

    def __init__(self):
        super().__init__()
        self.hosts = []
        self.addresses = []

    async def scan_host(self, address: str, master_port: int, timeout=1):
        """
        Scan a single SROS host and return a :class:`aztarna.sros.helpers.SROSHost` instance with all the data if found.

        :param address: Host IP address.
        :param master_port: Master node port.
        :param timeout: Timeout for the connection.
        :return: :class:`aztarna.sros.helpers.SROSHost` instance.
        """
        async with self.semaphore:
            sros_host = None
            logger.warning('Connecting to {}:{}'.format(address, master_port))
            try:
                master_address, port, master_cert = await get_sros_certificate(address, master_port, timeout)
                if master_cert:
                    if master_cert.subject['commonName'] == 'master':
                        sros_host = SROSHost()
                        sros_host.address = address
                        sros_host.port = master_port
                        master_node = get_node_info(master_cert)
                        master_node.port = master_port
                        master_node.policies = get_policies(master_cert)
                        sros_host.nodes.append(master_node)
                        results = []
                        if self.extended:
                            port_range = list(range(1024, 49151))
                            random.shuffle(port_range)
                            node_ports = await find_node_ports(address, port_range)
                            for port in node_ports:
                                results.append(get_sros_certificate(address, port))
                            for result in await asyncio.gather(*results):
                                try:
                                    if result:
                                        print(result)
                                        if result[2]:
                                            node_info = get_node_info(result[2])
                                            node_info.policies = get_policies(result[2])
                                            sros_host.nodes.append(node_info)
                                except Exception as e:
                                    logger.exception('Exception at host scan', e)
            except Exception:
                logger.exception('Exception at host scan')
                return None
        return sros_host

    async def scan_network(self):
        """
        Scan all the hosts specified in the internal hosts list :attr:`self.hosts`.

        :return: A list of :class:`aztarna.sros.helpers.SROSHost` containing all the found hosts.
        """
        try:
            results = []
            for port in self.ports:
                for host_address in self.host_list:
                    results.append(self.scan_host(host_address, port))
            for result in await asyncio.gather(*results):
                if result:
                    self.hosts.append(result)

        except AddressValueError:
            print('Invalid network entered')
        except Exception as e:
            print(e)

    def scan(self):
        """
        Run the scan for SROS hosts. Extended from :class:`aztarna.commons.BaseScanner`.
        This function is the one to be called externally in order to run the scans. Internally those scans are run
        with the help of asyncio.
        """
        asyncio.get_event_loop().run_until_complete(self.scan_network())

    async def scan_pipe(self):
        async for line in RobotAdapter.stream_as_generator(asyncio.get_event_loop(), sys.stdin):
            str_line = (line.decode()).rstrip('\n')
            for port in self.ports:
                await self.scan_host(str_line, port)

    def scan_pipe_main(self):
        asyncio.get_event_loop().run_until_complete(self.scan_pipe())

    def print_results(self):
        """
        Print the results of the scan into console. Extended from :class:`aztarna.commons.BaseScanner`.

        """
        for host in self.hosts:
            print('{}:{}'.format(host.address, host.port))
            for node in host.nodes:
                print('\tNode name: {}'.format(node.name))
                print('\tPort: {}'.format(node.port))
                print('\tDemo CA Used: {}'.format(node.is_demo))
                if self.extended:
                    print('\tPolicies:')
                    for policy in node.policies:
                        print('\t\tType: {}'.format(policy.type))
                        print('\t\tPermission: {}'.format(policy.permissions))
                        print('\t\tValues: ')
                        for value in policy.values:
                            print('\t\t\t{}'.format(value))
                        print('')
                print('')

    def write_to_file(self, out_file: str):
        """
        Write the results to the specified output file. Extended from :class:`aztarna.commons.BaseScanner`.

        :param out_file: Output file name in which the results will be writen.
        """
        lines = []
        header = 'Address;Node;Port;Demo;Policy_Type;Value;Permission\n'
        lines.append(header)
        for host in self.hosts:
            for node in host.nodes:
                for policy in node.policies:
                    for value in policy.values:
                        line = '{};{};{};{};{};{};{}\n'.format(host.address, node.name, node.port, node.is_demo,
                                                               policy.type, value, policy.permissions)
                        lines.append(line)
        with open(out_file, 'w') as file:
            file.writelines(lines)
