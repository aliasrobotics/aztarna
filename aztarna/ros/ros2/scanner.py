import os
import time
import pyshark
import threading
from typing import List

from aztarna.commons import RobotAdapter
from aztarna.ros.ros2.helpers import ROS2Node, ROS2Host, ROS2Topic, ROS2Service, raw_topics_to_pyobj_list, \
    raw_services_to_pyobj_list

# Max value of ROS_DOMAIN_ID
#   See https://github.com/eProsima/Fast-RTPS/issues/223
#   See https://answers.ros.org/question/318386/ros2-max-domain-id/
max_ros_domain_id = 232
rmw_implementations = [
        'rmw_opensplice_cpp',
        'rmw_fastrtps_cpp',
        'rmw_connext_cpp',
        'rmw_cyclonedds_cpp'
        ]


class ROS2Scanner(RobotAdapter):

    # Max value of ROS_DOMAIN_ID
    #   See https://github.com/eProsima/Fast-RTPS/issues/223
    #   See https://answers.ros.org/question/318386/ros2-max-domain-id/
    max_ros_domain_id = 232

    def __init__(self):
        super().__init__()
        self.found_hosts = []
        self.scanner_node_name = 'aztarna'

    @staticmethod
    def get_available_rmw_implementations():
        try:
            from ros2pkg.api import get_package_names
        except ImportError or ModuleNotFoundError:
            raise Exception('ROS2 needs to be installed and sourced to run ROS2 scans')
        packages = get_package_names()
        available_middlewares = []
        for pkg in packages:
            if pkg in rmw_implementations:
                available_middlewares.append(pkg)
        return available_middlewares

    def on_thread(self, domain_id):

        try:
            import rclpy
            from rclpy.context import Context
        except ImportError:
            raise Exception('ROS2 needs to be installed and sourced to run ROS2 scans')

        print("Exploring ROS_DOMAIN_ID: " + str(domain_id))
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)
        # available_middlewares = self.get_available_rmw_implementations()
        # for rmw in available_middlewares:
        #   os.environ['RMW_IMPLEMENTATION'] = rmw
        
        rclpy.init()
        
        # # Implementation based on rclpy has some issues have been detected
        # # when reproduced both in Linux and OS X. Essentially, calls to fetch nodes
        # # topics and services deliver incomplete information.        
        # scanner_node = rclpy.create_node(self.scanner_node_name)
        # found_nodes = self.scan_ros2_nodes(scanner_node)
        # print(len(found_nodes))
        # if found_nodes:
        #     host = ROS2Host()
        #     host.domain_id = domain_id
        #     host.nodes = found_nodes
        #     host.topics = self.scan_ros2_topics(scanner_node)
        #     print(len(host.topics))
        #     host.services = self.scan_ros2_services(scanner_node)
        #     print(len(host.services))
        #     if self.extended:
        #         for node in found_nodes:
        #             self.get_node_topics(scanner_node, node)
        #             self.get_node_services(scanner_node, node)
        #     self.found_hosts.append(host)

        # TODO: This implementation is a quick fix to cope with the limitations
        # identified in both OS X and Linux. Once this is cleared, code should revert to the above
        # simpler implementation
        node_count = 0
        topic_count = 0
        services_count = 0
        
        found_nodes = None
        found_topics = None
        found_services = None
        
        for i in range(10):
            scanner_node = rclpy.create_node(self.scanner_node_name)
            found_nodes_aux = self.scan_ros2_nodes(scanner_node)
            if (len(found_nodes_aux) > node_count):
                node_count = len(found_nodes_aux)
                found_nodes = found_nodes_aux
        
            if found_nodes_aux:
                host = ROS2Host()
                host.domain_id = domain_id
                host.nodes = found_nodes_aux
                found_topics_aux = self.scan_ros2_topics(scanner_node)
                # print("topics_aux found: "+str(len(found_topics_aux)))
                if (len(found_topics_aux) > topic_count):
                    topic_count = len(found_topics_aux)
                    found_topics = found_topics_aux
                found_services_aux = self.scan_ros2_services(scanner_node)
                if (len(found_services_aux) > services_count):
                    services_count = len(found_services_aux)
                    found_services = found_services_aux                
        
        # print("nodes: "+str(len(found_nodes)))
        # print("topics: "+str(len(found_topics)))
        # print("services: "+str(len(found_services)))
        if found_nodes:
            host = ROS2Host()
            host.domain_id = domain_id
            host.nodes = found_nodes
            host.topics = found_topics
            host.services = found_services
            if self.extended:
                for node in found_nodes:
                    self.get_node_topics(scanner_node, node)
                    self.get_node_services(scanner_node, node)
            self.found_hosts.append(host)
        rclpy.shutdown()

    def scan_pipe_main(self):
        raise NotImplementedError


    def print_results(self):
        """
        Print scanner results on stdout.
        """
        for host in self.found_hosts:
            print(f'[+] Host found in Domain ID {host.domain_id}')
            print('\tTopics:')
            if host.topics:
                for topic in host.topics:
                    print(f'\t\tTopic Name: {topic.name} \t|\t Topic Type: {topic.topic_type}')
            print('\tServices:')
            if host.services:
                for service in host.services:
                    print(f'\t\tService Name: {service.name} \t|\t Service Type: {service.service_type}')
            print('\tNodes:')
            if host.nodes:
                for node in host.nodes:
                    print(f'\t\tNode Name: {node.name} \t|\t Namespace: {node.namespace}')
                    if self.extended:
                        self.print_node_topics(node)
                print('-' * 80)

    @staticmethod
    def print_node_topics(node: ROS2Node):
        """
        Helper function for printing node related topics only.

        :param node: :class:`aztarna.ros.ros2.helpers.ROS2Node` containing the topics.
        """
        print(f'\t\tPublished topics:')
        for topic in node.published_topics:
            print(f'\t\t\tTopic Name: {topic.name} \t|\t Topic Type: {topic.topic_type}')
        print('\t\tSubscribed topics:')
        for topic in node.subscribed_topics:
            print(f'\t\t\tTopic Name: {topic.name} \t|\t Topic Type: {topic.topic_type}')

    @staticmethod
    def print_node_services(node: ROS2Node):
        """
        Helper function for printing node related services.

        :param node: :class:`aztarna.ros.ros2.helpers.ROS2Node` containing the serivices.
        """
        print(f'\t\tPublished topics:')
        for service in node.services:
            print(f'\t\t\tService Name: {service.name} \t|\t Service Type: {service.service_type}')

    def write_to_file(self, out_file: str):
        """
        Write scanner results to the specified output file.

        :param out_file: Output file to write the results on.
        """
        lines = []
        header = 'DomainID;NodeName;Namespace;Type;ElementName;ElementType;Direction\n'
        lines.append(header)
        with open(out_file, 'w') as f:
            for host in self.found_hosts:
                if self.extended:
                    for node in host.nodes:
                        self.write_node_topics(host, lines, node)
                        self.write_node_services(host, lines, node)
                else:
                    for topic in host.topics:
                        line = f'{host.domain_id};;;Topic;{topic.name};{topic.topic_type};;\n'
                        lines.append(line)
                    for service in host.services:
                        line = f'{host.domain_id};;;Service;{service.name};{service.service_type};;\n'
                        lines.append(line)
            f.writelines(lines)

    @staticmethod
    def write_node_topics(host: ROS2Host, lines: List[str], node: ROS2Node):
        """
        Helper function to generate the node related topic information lines for writing in file.

        :param host: :class:`aztarna.ros.ros2.helpers.ROS2Host` class object.
        :param lines: list containing the lines to write on the file.
        :param node: :class:`aztarna,.ros.ros2.helpers.ROS2Node` class object containing the information to write.
        """
        for published_topic in node.published_topics:
            line = f'{host.domain_id};{node.name};{node.namespace};{published_topic.name};' \
                f'{published_topic.topic_type};Publish\n'
            lines.append(line)
        for subscribed_topic in node.subscribed_topics:
            line = f'{host.domain_id};{node.name};{node.namespace};{subscribed_topic.name};' \
                f'{subscribed_topic.topic_type};Subscribe\n'
            lines.append(line)

    @staticmethod
    def write_node_services(host: ROS2Host, lines: List[str], node: ROS2Node):
        """
        Helper function to generate the node related service information lines for writing in file.

        :param host: :class:`aztarna.ros.ros2.helpers.ROS2Host` class object.
        :param lines: list containing the lines to write on the file.
        :param node: :class:`aztarna,.ros.ros2.helpers.ROS2Node` class object containing the information to write.
        """
        for service in node.services:
            line = f'{host.domain_id};{node.name};{node.namespace};Service;{service.name};' \
                f'{service.service_type};\n'
            lines.append(line)

    def scan(self) -> List[ROS2Host]:
        """
        Scan the local network and all available ROS_DOMAIN_IDs against ROS2 nodes.

        :return: A list containing the found ROS2 systems.
        """
<<<<<<< HEAD

        # Run passive mode using interface passed over argument
        if self.passive is not None:
            self.scan_passive(self.passive)
            return

        try:
            import rclpy
            from rclpy.context import Context
        except ImportError:
            raise Exception('ROS2 needs to be installed and sourced to run ROS2 scans')

        # Explore the specified domain or all depending on the arguments provided (-d option)
        # TODO: consider ranges (e.g. 1-5) if provided
=======
>>>>>>> 78ea2e5a1116253f333a904efdda5db1e3c1772b
        domain_id_range_init = 0
        domain_id_range_end = max_ros_domain_id
        domain_id_range = range(domain_id_range_init, domain_id_range_end)

        # Choose domain id
        if self.domain is not None:
            domain_id_range = [self.domain]
        else:
            print("Exploring ROS_DOMAIN_ID from: "+str(domain_id_range_init)+str(" to ")+str(domain_id_range_end))
        print('Scanning the network...')
        threads = []
        for i in domain_id_range:
            t = threading.Thread(self.on_thread(i))
            threads.append(t)
            t.start()
        return self.found_hosts

    def scan_ros2_nodes(self, scanner_node) -> List[ROS2Node]:
        """
        Helper function to scan the nodes on a certain domain.

        :param scanner_node: Scanner node object to be used for retrieving the node information.
        :return: A list containing the found nodes.
        """
        nodes = scanner_node.get_node_names_and_namespaces()
        found_nodes = []
        for node_name, namespace in nodes:
            if node_name != self.scanner_node_name:
                found_node = ROS2Node()
                found_node.name = node_name
                found_node.namespace = namespace
                found_nodes.append(found_node)
        return found_nodes

    @staticmethod
    def scan_ros2_topics(scanner_node) -> List[ROS2Topic]:
        """
        Helper function for scanning ROS2 topic related information.

        :param scanner_node: Scanner node object to be used for retrieving the node information.
        :return: List containing the found topics.
        """
        topics = scanner_node.get_topic_names_and_types()
        return raw_topics_to_pyobj_list(topics)

    @staticmethod
    def scan_ros2_services(scanner_node) -> List[ROS2Service]:
        """
        Helper function for scanning ROS2 Service related information.

        :param scanner_node:
        :return: List of :class:`aztarna.ros.ros2.helpers.ROS2Service`
        """
        services = scanner_node.get_service_names_and_types()
        return raw_services_to_pyobj_list(services)

    @staticmethod
    def get_node_topics(scanner_node, node: ROS2Node):
        """
        Get available topics for a certain node, detailing if the node is publishing or subscribing to them.

        :param scanner_node: Scanner node object to be used for retrieving the node information.
        :param node: Target :class:`aztarna.ros.ros2.helpers.ROS2Node`
        """
        published_topics = scanner_node.get_publisher_names_and_types_by_node(node.name, node.namespace)
        subscribed_topics = scanner_node.get_subscriber_names_and_types_by_node(node.name, node.namespace)
        node.published_topics = raw_topics_to_pyobj_list(published_topics)
        node.subscribed_topics = raw_topics_to_pyobj_list(subscribed_topics)

    @staticmethod
    def get_node_services(scanner_node, node: ROS2Node):
        """
        Get services provided by a certain node.

        :param scanner_node: Scanner node object to be used for retrieving the node information.
        :param node: Target :class:`aztarna.ros.ros2.helpers.ROS2Node`
        """
        services = scanner_node.get_service_names_and_types_by_node(node.name, node.namespace)
        node.services = raw_services_to_pyobj_list(services)

    @staticmethod
    def scan_passive(interface: str):
        for pkg in pyshark.LiveCapture(interface=interface, display_filter='rtps'):
            print(pkg)
