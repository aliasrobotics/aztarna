#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS Scanner module.

:author: Gorka Olalde Mendia(@olaldiko), Xabier Perez Baskaran(@xabierpb)
"""
import asyncio
import aiohttp
import logging
import re
from aiohttp_xmlrpc.client import ServerProxy
from aztarna.ros.commons import CommunicationROS
from aztarna.commons import RobotAdapter
from aztarna.ros.helpers import HelpersROS
from aztarna.ros.ros.helpers import Node, Topic, Service
from aztarna.ros.ros.helpers import ROSHost
import sys
from ipaddress import IPv4Address

class ROSScanner(RobotAdapter):
    """
    ROSScanner class, an extension of BaseScanner for ROS.
    """
    def __init__(self):
        super().__init__()

        self.timeout = aiohttp.ClientTimeout(total=3)
        self.hosts = []

        self.logger = logging.getLogger(__name__)

    async def analyze_nodes(self, address, port):
        """
        Scan a node and gather all its data including topics, services and Communications.

        :param address: address of the ROS master
        :param port: port of the ROS master
        """
        async with aiohttp.ClientSession(loop=asyncio.get_event_loop(), timeout=self.timeout) as client:
            full_host = 'http://' + str(address) + ':' + str(port)
            ros_master_client = ServerProxy(full_host, loop=asyncio.get_event_loop(), client=client)
            ros_host = ROSHost(address, port)
            async with self.semaphore:
                try:
                    code, msg, val = await ros_master_client.getSystemState('')
                    if code == 1:
                        self.hosts.append(ros_host)
                        if self.extended:
                            publishers_array = val[0]
                            subscribers_array = val[1]
                            services_array = val[2]
                            found_topics = await self.analyze_topic_types(ros_master_client)  # In order to analyze the nodes topics are needed

                            self.extract_nodes(publishers_array, found_topics, 'pub', ros_host)
                            self.extract_nodes(subscribers_array, found_topics, 'sub', ros_host)
                            self.extract_services(services_array, ros_host)

                            for topic_name, topic_type in found_topics.items():  # key, value
                                current_topic = Topic(topic_name, topic_type)
                                comm = CommunicationROS(current_topic)
                                for node in ros_host.nodes:
                                    if next((x for x in node.published_topics if x.name == current_topic.name), None) is not None:
                                        comm.publishers.append(node)
                                    if next((x for x in node.subscribed_topics if x.name == current_topic.name), None) is not None:
                                        comm.subscribers.append(node)
                                ros_host.communications.append(comm)
                            await self.set_xmlrpcuri_node(ros_master_client, ros_host)
                        await client.close()
                        self.logger.warning('[+] ROS Host found at {}:{}'.format(ros_host.address, ros_host.port))
                    else:
                        self.logger.critical('[-] Error getting system state. Probably not a ROS Master')

                except Exception as e:
                    # traceback.print_tb(e.__traceback__)
                    self.logger.error('[-] Error connecting to host ' + str(ros_host.address) + ':' + str(ros_host.port) + ' -> '+str(e) + '\n\tNot a ROS host')

    def extract_nodes(self, source_array, topics, pub_or_sub, host):
        """
        From all the data ROS Master returns, extract just the node info.

        :param source_array: A multiple level array containing data from the the ROS system state
        :param topics: A list of all topics found in the ROS system
        :param pub_or_sub: A boolean to separate publisher and subscriber nodes
        :param host: Current ROS host
        """
        source_lines = list(map(HelpersROS.process_line, list(filter(lambda x: (list(x)) is not None, source_array))))
        for source_line in source_lines:
            for node_name in source_line[1]:  # source_line[1] == nodes from a topic, is a list
                node = self.get_create_node(node_name, host)
                topic_name = source_line[0]
                topic_type = topics[topic_name]
                topic = Topic(topic_name, topic_type)
                if topic not in node.published_topics and pub_or_sub == 'pub':
                    node.published_topics.append(topic)
                if topic not in node.subscribed_topics and pub_or_sub == 'sub':
                    node.subscribed_topics.append(topic)

    @staticmethod
    def get_create_node(node_name, host):
        """
        Generate new :class:`aztarna.ros.helpers.Node` objects, and if they exist just return them.

        :param node_name: The name of the node to create or return
        :param host: Current ROS host
        :return: The newly created node or an existing that matches :attr:`node_name`
        """
        node_name_attrs = [o.name for o in host.nodes]
        if node_name not in node_name_attrs:
            ret_node = Node(node_name)
            host.nodes.append(ret_node)
        else:
            ret_node = next((x for x in host.nodes if x.name == node_name), None)

        return ret_node

    async def set_xmlrpcuri_node(self, ros_master_client, host):
        """
        Once all node data is collected, set the xml.

        :param ros_master_client: xml-rpc object for the ROS Master Client
        """
        for node in host.nodes:
            uri = await ros_master_client.lookupNode('', node.name)
            if uri[2] != '':
                regexp = re.compile(r'http://(?P<host>\S+):(?P<port>[0-9]{1,5})')
                uri_groups = regexp.search(uri[2])
                node.address = uri_groups.group('host')
                node.port = uri_groups.group('port')

    @staticmethod
    async def analyze_topic_types(ros_master_client):
        """
        Extract topic from ROS Master and disassemble them into topic name and topic type.

        :param ros_master_client:  xml-rpc object for the ROS Master Client
        :return: A dictionary of topics. Key is the topic name and value the topic type
        """
        topic_types = await ros_master_client.getTopicTypes('')
        topics = {}
        for topic_type_element in topic_types[2]:
            topic_name = topic_type_element[0]
            topic_type = topic_type_element[1]
            topics[topic_name] = topic_type
        return topics

    def extract_services(self, source_array, host):
        """
        Extract the services from the ROS system state.

        :param source_array: A multiple level array containing data from the the ROS system state
        :param host: Current ROS host
        """
        service_lines = list(map(HelpersROS.process_line, list(filter(lambda x: (list(x)) is not None, source_array))))
        for service_line in service_lines:
            for node_name in service_line[1]:  # source_line[1] == nodes from a topic, is a list
                node = self.get_create_node(node_name, host)
                node.services.append(Service(service_line[0]))

    async def scan_network(self):
        """
        Scan the provided network (from args) searching for ROS nodes.
        """
        try:
            results = []
            for port in self.ports:
                for address in self.host_list:
                    results.append(self.analyze_nodes(address, port))

            for result in await asyncio.gather(*results):
                pass

        except ValueError as e:
            self.logger.error('Invalid address entered')
            raise e

    def scan(self):
        """
        Call to :meth:`aztarna.ros.scanner.scan_network` asynchronously
        """
        asyncio.get_event_loop().run_until_complete(self.scan_network())

    async def scan_pipe(self):
        async for line in RobotAdapter.stream_as_generator(asyncio.get_event_loop(), sys.stdin):
            str_line = (line.decode()).rstrip('\n')
            for port in self.ports:
                await self.analyze_nodes(str_line, port)

    def scan_pipe_main(self):
        asyncio.get_event_loop().run_until_complete(self.scan_pipe())

    def print_results(self):
        """
        Print the information of a ROS system.
        """
        for host in self.hosts:
            for node in host.nodes:
                print('\nNode: ' + str(node))
                print('\n\t Published topics:')
                for topic in node.published_topics:
                    print('\t\t * ' + str(topic))
                print('\n\t Subscribed topics:')
                for topic in node.subscribed_topics:
                    print('\t\t * ' + str(topic))
                print('\n\t Services:')
                for service in node.services:
                    print('\t\t * ' + str(service))

            for i in range(len(host.communications)):
                comm = host.communications[i]
                print('\n\t CommunicationROS ' + str(i) + ':')
                print('\t\t - Publishers:')
                for node in comm.publishers:
                    print('\t\t\t' + str(node))
                print('\t\t - Topic: ' + str(comm.topic))
                print('\t\t - Subscribers:')
                for node in comm.subscribers:
                    print('\t\t\t' + str(node))
            print('\n\n')

    def write_to_file(self, out_file):
        """
        Write the information of a ROS system into the provided file.

        :param out_file: The file where to write the results
        """
        lines = []
        header = 'Master Address;Master port;Node Name;Node Address;Port;Published Topics;Subscribed Topics;Services\n'
        lines.append(header)
        for host in self.hosts:
            line = '{};{};;;;;;\n'.format(host.address, host.port)
            if len(host.nodes) > 0:
                for node in host.nodes:
                    for ptopic in node.published_topics:
                        for stopic in node.subscribed_topics:
                            for service in node.services:
                                line = '{};{};{};{};{};{};{};{}\n'.format(host.address, host.port, node.name, node.address, node.port, ptopic,
                                                                   stopic, service)
            lines.append(line)

        with open(out_file, 'w') as file:
            file.writelines(lines)
