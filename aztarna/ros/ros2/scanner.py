import os
from collections import namedtuple
import pyshark
import threading
from typing import List

try:
    from ros2cli.node.strategy import add_arguments, NodeStrategy
    from ros2node.api import get_node_names, get_publisher_info, NodeNameCompleter
    from ros2node.verb import VerbExtension
    from ros2cli.node.direct import DirectNode
    from rclpy.action.graph import get_action_client_names_and_types_by_node, get_action_server_names_and_types_by_node
    from ros2node.api import get_service_info, get_subscriber_info
    from ros2node.verb.info import print_names_and_types
    from ros2topic.api import get_topic_names_and_types
    from ros2service.api import get_service_names_and_types
    from ros2service.verb import VerbExtension
except ImportError:
    # NOTE: from ROS 2 Eloquent on:
    #   it can't find get_service_info anymore due to API changes. See
    #   https://github.com/ros2/ros2cli/commit/e020486c82cdee1c8d622f5c88fe564ef1b564e7#diff-4602044fc49daf9e20cd8c4ff8d27923
    #   which essentially removes the get_service_info primitive
    from ros2cli.node.strategy import add_arguments, NodeStrategy
    from ros2node.api import get_node_names, get_publisher_info, NodeNameCompleter
    from ros2node.verb import VerbExtension
    from ros2cli.node.direct import DirectNode
    from rclpy.action.graph import get_action_client_names_and_types_by_node, get_action_server_names_and_types_by_node
    from ros2node.api import get_subscriber_info
    from ros2node.api import get_service_client_info as get_service_info
    from ros2node.verb.info import print_names_and_types
    from ros2topic.api import get_topic_names_and_types
    from ros2service.api import get_service_names_and_types
    from ros2service.verb import VerbExtension
except ModuleNotFoundError:
    # generally when ROS 2 is missing
    print('ROS2 needs to be installed and sourced to run ROS2 scans')

from aztarna.commons import RobotAdapter
from aztarna.ros.ros2.helpers import ROS2Node, ROS2Host, ROS2Topic, ROS2Service, raw_topics_to_pyobj_list, \
    raw_services_to_pyobj_list

# Max value of ROS_DOMAIN_ID
#   See https://github.com/eProsima/Fast-RTPS/issues/223
#   See https://answers.ros.org/question/318386/ros2-max-domain-id/
max_ros_domain_id = 231
rmw_implementations = [
        'rmw_opensplice_cpp',
        'rmw_fastrtps_cpp',
        'rmw_connext_cpp',
        'rmw_cyclonedds_cpp'
        ]

default_topics = ['/rosout', '/parameter_events']
NodeName = namedtuple('NodeName', ('name', 'namespace', 'full_name'))
TopicInfo = namedtuple('Topic', ('name', 'types'))
ServiceInfo = namedtuple('Service', ('name', 'types'))

# Helper functions for the use of the ros2cli daemon.
# Fetched from ros2node.api.__init__


def parse_node_name(node_name):
    full_node_name = node_name
    if not full_node_name.startswith('/'):
        full_node_name = '/' + full_node_name
    namespace, node_basename = full_node_name.rsplit('/', 1)
    if namespace == '':
        namespace = '/'
    return NodeName(node_basename, namespace, full_node_name)


def get_action_server_info(*, node, remote_node_name):
    remote_node = parse_node_name(remote_node_name)
    names_and_types = get_action_server_names_and_types_by_node(
        node, remote_node.name, remote_node.namespace)
    return [
        TopicInfo(
            name=n,
            types=t)
        for n, t in names_and_types]


def get_action_client_info(*, node, remote_node_name):
    remote_node = parse_node_name(remote_node_name)
    names_and_types = get_action_client_names_and_types_by_node(
        node, remote_node.name, remote_node.namespace)
    return [
        TopicInfo(
            name=n,
            types=t)
        for n, t in names_and_types]


class ROS2Scanner(RobotAdapter):

    def __init__(self):
        super().__init__()
        self.found_hosts = []
        self.scanner_node_name = 'aztarna'

        # Two alternatives were considered here, either using classes to abstract
        # the different ROS 2 abstractions (implemented in helpers2.py) or
        # a simple list[dict]. The second option was selected for brevity.

        # a list of ROS 2 nodes abstracted as dictionaries, where each key
        # represents a the corresponding element within a node and the value
        # captures its values. In case of several elements within a value,
        # a list or alternative another dict might be created
        #
        # For clarity, an example instance is described below:
        #     self.processed_nodes =
        #     [ {
        #         "name": "/listener",
        #         "domain": 0,
        #         "namespace": "/",
        #         "subscribers": [Topic(name='/parameter_events', types=['rcl_interfaces/msg/ParameterEvent']),
        #                         Topic(name='/rosout', types=['rcl_interfaces/msg/Log'])],
        #         "publishers": [...],
        #         "services": [],
        #         "actionservers": [],
        #         "actionclients": [],
        #         ...
        #       },
        #       ...
        #     ]
        self.processed_nodes = []
        self.processed_topics = []
        self.processed_services = []

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

    def ros2cli_api(self, domain_id):
        """
        Invokes the different methods of the ros2cli API to fetch abstractions
        from the ROS 2 graph and populates self.processed_nodes
        """
        self.ros2node(domain_id)
        self.ros2topic(domain_id)
        self.ros2service(domain_id)

    def ros2node(self, domain_id):
        """
        'ros2 node list' fetched from ros2cli and more
        """
        print("Exploring ROS_DOMAIN_ID: " + str(domain_id) + ' for nodes')
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)

        with NodeStrategy("-a") as node:
            node_names = get_node_names(node=node, include_hidden_nodes="-a")

        nodes = sorted(n.full_name for n in node_names)
        output_node = {}
        for nodo in nodes:
            # # Abstractions from helper2
            # output_node = ROS2node()
            # output_node.name = nodo
            # output_node.domain_id = domain_id
            # output_node.namespace = output_node.name[:(output_node.name.rfind("/") + 1)] # fetch the substring until the last "/"

            # Abstractions using a list[dict] as defined above (see self.processed_nodes):
            #output_node = {"name": nodo, "domain": domain_id}
            #output_node["namespace"] = output_node["name"][:(output_node["name"].rfind("/") + 1)] # fetch the substring until the last "/"

            with DirectNode(nodo) as node:
                if str(nodo) != '/_ros2cli_daemon_0':
                    output_node[nodo] = {
                                          'domain': domain_id,
                                          'subscribers': get_subscriber_info(node=node, remote_node_name=nodo),
                                          'publishers': get_publisher_info(node=node, remote_node_name=nodo),
                                          'services': get_service_info(node=node, remote_node_name=nodo),
                                          'actionservers': get_action_server_info(node=node, remote_node_name=nodo),
                                          'actionclient': get_action_client_info(node=node, remote_node_name=nodo)
                                        }
        self.processed_nodes.append(output_node)

    def ros2topic(self, domain_id):

        print("Exploring ROS_DOMAIN_ID: " + str(domain_id) + ' for topics')
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)

        with NodeStrategy('-t') as node:
            topic_names_and_types = get_topic_names_and_types(node=node)

        output_topic = {}
        if topic_names_and_types:
            for (topic_name, topic_types) in topic_names_and_types:
                with DirectNode(topic_name) as node:
                    if topic_name not in default_topics:
                        output_topic[topic_name] = {
                                                    'domain': domain_id,
                                                    'type': str(topic_types).replace('[', '').replace(']', ''),
                                                    'publishers': node.count_publishers(topic_name),
                                                    'subscriber': node.count_subscribers(topic_name)
                                                    }
            self.processed_topics.append(output_topic)

    def ros2service(self, domain_id):
        print('Exploring ROS_DOMAIN_ID: ' + str(domain_id) + ' for services')
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)

        with NodeStrategy('-t') as node:
            service_names_and_types = get_service_names_and_types(
                node=node)
        output_service = {}
        if service_names_and_types:
            for (service_name, service_types) in service_names_and_types:
                output_service[service_name] = {'domain': domain_id,
                                                'type': str(service_types).replace('[', '').replace(']', '')
                                                }

        self.processed_services.append(output_service)

    def on_thread(self, domain_id):
        """
        Calls rclpy methods to implement a quick way to footprint the network

        TODO: @LanderU creates a single host, this is wrong, should be reviewed.
        """
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

        # Implementation based on rclpy has some issues have been detected
        # when reproduced both in Linux and OS X. Essentially, calls to fetch nodes
        # topics and services deliver incomplete information.
        rclpy.init()
        try:
            scanner_node = rclpy.create_node(self.scanner_node_name)
            found_nodes = self.scan_ros2_nodes(scanner_node)
            if found_nodes:
                host = ROS2Host()
                host.domain_id = domain_id
                host.nodes = found_nodes
                host.topics = self.scan_ros2_topics(scanner_node)
                host.services = self.scan_ros2_services(scanner_node)
                if self.extended:
                    for node in found_nodes:
                        self.get_node_topics(scanner_node, node)
                        self.get_node_services(scanner_node, node)
                self.found_hosts.append(host)
            rclpy.shutdown()

        except:
            raise Exception("Security plugins not supported in aztarna!")

    def scan_pipe_main(self):
        raise NotImplementedError

    @staticmethod
    def print_node_topics(node: ROS2Node):
        """
        Helper function for printing node related topics only.

        :param node: :class:`aztarna.ros.ros2.helpers.ROS2Node` containing the topics.
        """
        print(f'\t\tPublished topics:')  # noqa: E999
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

    def scan_passive(self, interface: str):
        for pkg in pyshark.LiveCapture(interface=interface, display_filter='rtps'):
            print(pkg)

    def print_results(self):
        """
        Print scanner results on stdout.
        """
        if self.use_daemon:
            self.print_results_daemon()
        else:
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

    def print_results_daemon(self):
        """
        Helper method to print information fetched using rclpy ros2cli daemon
        """
        print('Nodes: ')
        for node in self.processed_nodes:
            for node_key, node_values in node.items():
                # Apply filters to the print_results by default
                if not(self.hidden):
                    if "_ros2cli" in node_key:
                        continue

                print('\t' + str(node_key))
                # Iterate over node sub-elements
                for elem_key, elem_value in node_values.items():
                    print('\t\t' + str(elem_key) + ": " + str(elem_value))
        print('Topics: ')
        for topic in self.processed_topics:
            for key in topic:
                print('\t' + key)
                print('\t\t ' + str(str(topic[key]).replace(',', '\n\t\t')).replace('{', '').replace('}', '').replace('\'', ''))
        print('Services: ')
        for service in self.processed_services:
            for key in service:
                print('\t' + key)
                print('\t\t ' + str(str(service[key]).replace(',', '\n\t\t')).replace('{', '').replace('}', '').replace('\'', ''))

    def scan(self) -> List[ROS2Host]:
        """
        Scan the local network and all available ROS_DOMAIN_IDs against ROS2 nodes.

        :return: A list containing the ROS2 abstractions found.
        """
        domain_id_range_init = 0
        domain_id_range_end = max_ros_domain_id
        domain_id_range = range(domain_id_range_init, domain_id_range_end)

        if self.passive:
            # Launch a passive scan for RTPS packages in all network interfaces
            # TODO: pass the corresponding provided interface if matches with
            #     available ones. Capture errors and process them as well.
            self.scan_passive("any")

        if self.domain is not None:
            domain_id_range = [self.domain]
        else:
            print("Exploring ROS_DOMAIN_ID from: " + str(domain_id_range_init) + str(" to ") + str(domain_id_range_end))

        print('Scanning the network...')
        threads = []
        for i in domain_id_range:
            if self.use_daemon:
                # This approach does the following:
                #   1. calls ros2cli ros2node and populates abstractions from helper2,
                #       dumping them into self.processed_nodes as it applies
                #   2. calls ros2cli ros2topic and populates
                #   3. calls ros2cli ros2service and populates

                # NOTE: scanning all the domains can take several minutes
                t = threading.Thread(self.ros2cli_api(i))
                threads.append(t)
                t.start()
                # self.print_results_daemon()
            else:
                # This approach does the following:
                #   1. calls rclpy scan_ros2_nodes and populates abstractons from helper
                #   2. calls rclpy scan_ros2_topics and populates
                #   3. calls rclpy scan_ros2_services and populates

                # NOTE: scanning all the domains takes only a few seconds
                t = threading.Thread(self.on_thread(i))
                threads.append(t)
                t.start()

        return self.found_hosts
