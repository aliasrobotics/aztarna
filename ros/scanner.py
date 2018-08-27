import re
from ipaddress import ip_network, IPv4Address
from commons import BaseScanner, Communication
from helpers import HelpersROS, FileUtils
from ros.helpers import Node, Topic, Service
from xmlrpc.client import ServerProxy
import logging
import socket


class ROSScanner(BaseScanner):

    def __init__(self):
        super().__init__()

        self.nodes = []
        self._communications = []
        self._parameters = []

        self.logger = logging.getLogger(__name__)

    def analyze_nodes(self, host):  # similar to scan_
        ros_master_client = ServerProxy(host)
        socket.setdefaulttimeout(2)

        try:
            code, msg, val = ros_master_client.getSystemState('')

            if code == 1:
                publishers_array = val[0]
                subscribers_array = val[1]
                services_array = val[2]
                found_topics = self.analyze_topic_types(ros_master_client)  # In order to analyze the nodes topics are needed

                self.extract_nodes(publishers_array, found_topics, 'pub')
                self.extract_nodes(subscribers_array, found_topics, 'sub')
                self.extract_services(services_array)

                for topic_name, topic_type in found_topics.items():  # key, value
                    current_topic = Topic(topic_name, topic_type)
                    comm = Communication(current_topic)
                    for node in self.nodes:
                        if next((x for x in node.published_topics if x.name == current_topic.name), None) is not None:
                            comm.publishers.append(node)
                        if next((x for x in node.subscribed_topics if x.name == current_topic.name), None) is not None:
                            comm.subscribers.append(node)
                    self._communications.append(comm)

                self.set_xmlrpcuri_node(ros_master_client)
            else:
                self.logger.critical('[-] Error getting system state')

        except Exception as e:
            self.logger.error('[-] Error connecting to host ' + str(host) + ': ' + str(e))

    def extract_nodes(self, source_array, topics, pub_or_sub):
        source_lines = list(map(HelpersROS.process_line, list(filter(lambda x: (list(x)) is not None, source_array))))
        for source_line in source_lines:
            for node_name in source_line[1]:  # source_line[1] == nodes from a topic, is a list
                node = self.get_create_node(node_name)
                topic_name = source_line[0]
                topic_type = topics[topic_name]
                topic = Topic(topic_name, topic_type)
                if topic not in node.published_topics and pub_or_sub == 'pub':
                    node.published_topics.append(topic)
                if topic not in node.subscribed_topics and pub_or_sub == 'sub':
                    node.subscribed_topics.append(topic)

    def get_create_node(self, node_name):
        ret_node = None
        node_name_attrs = [o.name for o in self.nodes]
        if node_name not in node_name_attrs:
            ret_node = Node(node_name)
            self.nodes.append(ret_node)
        else:
            ret_node = next((x for x in self.nodes if x.name == node_name), None)

        return ret_node

    def set_xmlrpcuri_node(self, ros_master_client):
        for node in self.nodes:
            uri = ros_master_client.lookupNode('', node.name)[2]
            if uri != '':
                regexp = re.compile(r'http://(?P<host>[a-zA-Z\.{0-4}0-9]+):(?P<port>[0-9]{1,5})')
                uri_groups = regexp.search(uri)
                node.address = uri_groups.group('host')
                node.port = uri_groups.group('port')

    @staticmethod
    def analyze_topic_types(ros_master_client):
        topic_types = ros_master_client.getTopicTypes('')[2]
        topics = {}
        for topic_type_element in topic_types:
            topic_name = topic_type_element[0]
            topic_type = topic_type_element[1]
            topics[topic_name] = topic_type

        return topics

    def extract_services(self, source_array):
        service_lines = list(map(HelpersROS.process_line, list(filter(lambda x: (list(x)) is not None, source_array))))
        for service_line in service_lines:
            for node_name in service_line[1]:  # source_line[1] == nodes from a topic, is a list
                node = self.get_create_node(node_name)
                node.services.append(Service(service_line[0]))

    def load_from_file(self, filename):
        self.net_range = FileUtils.load_from_file(filename)

    def scan(self):
        try:
            network = ip_network(self.net_range)
            if network.netmask == IPv4Address('255.255.255.255'):
                host_list = [IPv4Address(self.net_range)]
            else:
                host_list = list(network.hosts())
            for address in host_list:
                full_host = 'http://' + str(address) + ':' + str(self.port)
                self.analyze_nodes(full_host)
        except ValueError as e:
            self.logger.error('Invalid address entered')
            raise e

    def print_results(self):
        for node in self.nodes:
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

        print('\nCommunications:')
        for i in range(len(self._communications)):
            comm = self._communications[i]
            print('\n\t Communication ' + str(i) + ':')
            print('\t\t - Publishers:')
            for node in comm.publishers:
                print('\t\t\t' + str(node))
            print('\t\t - Topic: ' + str(comm.topic))
            print('\t\t - Subscribers:')
            for node in comm.subscribers:
                print('\t\t\t' + str(node))
        print('\n\n')

    def write_to_file(self, out_file):
        pass


