#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio
import random
import traceback
from ipaddress import ip_network, IPv4Address, AddressValueError

from aztarna.commons import BaseScanner
from .helpers import SROSHost, get_node_info, get_policies, get_sros_certificate, find_node_ports


class SROSScanner(BaseScanner):

    def __init__(self):
        super().__init__()
        self.hosts = []
        self.addresses = []

    async def scan_host(self, address, master_port, timeout=1):
        sros_host = None
        master_address, port, master_cert = await get_sros_certificate(address, master_port, timeout)
        if master_cert:
            sros_host = SROSHost()
            sros_host.address = address
            master_node = get_node_info(master_cert)
            master_node.policies = get_policies(master_cert)
            sros_host.nodes.append(master_node)
            results = []
            if self.extended:
                port_range = list(range(11310, 25000))
                random.shuffle(port_range)
                node_ports = await find_node_ports(address, port_range)
                for port in node_ports:
                    results.append(get_sros_certificate(address, port))

                for result in await asyncio.gather(*results):
                    try:
                        print(result)
                        if result[2]:
                            node_info = get_node_info(result[2])
                            node_info.policies = get_policies(result[2])
                            sros_host.nodes.append(node_info)
                    except Exception as e:
                        print(e)

        return sros_host

    async def scan_network(self):
        sem = sem = asyncio.Semaphore(4000)
        try:
            for host_address in self.host_list:
                print('Scanning node {}'.format(host_address))
                async with sem:
                    sros_host = await self.scan_host(host_address, self.ports[0])  # TODO add port range
                    if sros_host:
                        self.hosts.append(sros_host)
        except AddressValueError:
            print('Invalid network entered')
        except Exception as e:
            traceback.print_tb(e)
            print(e)

    def scan(self):
        asyncio.get_event_loop().run_until_complete(self.scan_network())

    def print_results(self):
        for host in self.hosts:
            print(host.address)
            for node in host.nodes:
                print('\tNode name: {}'.format(node.name))
                print('\tPort: {}'.format(node.port))
                print('\tDemo CA Used: {}'.format(node.is_demo))
                print('\tPolicies:')
                for policy in node.policies:
                    print('\t\tType: {}'.format(policy.type))
                    print('\t\tPermission: {}'.format(policy.permissions))
                    print('\t\tValues: ')
                    for value in policy.values:
                        print('\t\t\t{}'.format(value))
                    print('')
                print('')

    def write_to_file(self, out_file):
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




