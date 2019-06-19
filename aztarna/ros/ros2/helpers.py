from typing import List

default_topics = ['/rosout', '/parameter_events']


class ROS2Node:

    def __init__(self):
        self.name = ''
        self.domain_id = 0
        self.namespace = ''
        self.subscribed_topics = []
        self.published_topics = []
        self.services = []


class ROS2Topic:

    def __init__(self):
        self.name = ''
        self.topic_type = ''
        self.domain_id = 0


class ROS2Host:

    def __init__(self):
        self.nodes = []
        self.topics = []
        self.domain_id = 0


class ROS2Service:

    def __init__(self):
        self.name = ''
        self.service_type = ''


def raw_topics_to_pyobj_list(topics, include_default=False) -> List[ROS2Topic]:
    """
    Utility function for converting the raw data returned by the ROS2 API into a list of python topic objects.

    :param topics: Raw topics list.
    :param include_default: If to include the default topic names on the returned list or not.
    :return: A list containing all parsed :class:`aztarna.ros.ros2.helpers.ROS2Topic` objects.
    """
    topics_list = []
    for topic_name, topic_type in topics:
        if not (not include_default and topic_name in default_topics):
            topic = ROS2Topic()
            topic.name = topic_name
            topic.topic_type = topic_type
            topics_list.append(topic)
    return topics_list


def raw_services_to_pyobj_list(services) -> List[ROS2Service]:
    """
    Utility function for converting the raw data returned by the ROS2 API into a list of python service objects.

    :param services: Raw services list.
    :return: A list containing all parsed :class:`aztarna.ros.ros2.helpers.ROS2Service` objects.
    """
    services_list = []
    for service_name, service_type in services:
        service = ROS2Service()
        service.name = service_name
        service.service_type = service_type
        services_list.append(service)
    return services_list
