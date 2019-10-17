from typing import List

default_topics = ['/rosout', '/parameter_events']


class ROS2host:
    """
    A class abstracting a host running ROS 2
    
    NOTE: this class isn't likely to be used since the ROS 2 api does 
    not expose the hosts by default.
    """
    def __init__(self):
        self.nodes = []
        self.topics = []
        self.domain_id = 0


class ROS2node:
    """
    A class abstracting a ROS 2 node
    """
    def __init__(self):
        self.name = ''
        self.domain_id = 0
        self.namespace = ''   
        self.subscribers = [] #  list of nodes
        self.publishers = [] # list of nodes
        self.topics = []
        self.services = []
        self.action_servers = []
        self.action_clients = [] 
        
    def __str__(self):
        return_str = ""
        return_str += "Node:\n"
        return_str += "\t Name: "+str(self.name)+"\n"
        return_str += "\t ROS2_DOMAIN_ID: "+str(self.domain_id)+"\n"
        return_str += "\t Namespace: "+str(self.namespace)+"\n"
        return return_str


class ROS2topic:
    """
    A class abstracting a ROS 2 topic
    """
    def __init__(self):
        self.name = ''
        self.topic_type = ''
        self.domain_id = 0
        self.subscribers = 0
        self.publishers = 0

    def __str__(self):
        return_str = ''
        return_str += 'Topic:\n'
        return_str += '\t Name: ' + str(self.name) + '\n'
        return_str += '\t ROS2_DOMAIN_ID: ' + str(self.domain_id) + '\n'
        return_str += '\t Number of publishers: ' + str(self.publishers) + '\n'
        return_str += '\t Number of subscribers: ' + str(self.publishers) + '\n'
        return_str += "\t Namespace: " + str(self.namespace) + "\n"
        return return_str


class ROS2service:
    """
    A class abstracting a ROS 2 service
    """
    def __init__(self):
        self.name = ''
        self.domain_id = 0
        self.service_type = ''

    def __str__(self):
        return_str = ''
        return_str += 'Services:\n'
        return_str += '\t Name: ' + str(self.name) + '\n'
        return_str += '\t ROS2_DOMAIN_ID: ' + str(self.domain_id) + '\n'
        return_str += '\t Service type: ' + self.service_type() + '\n'
        return return_str


class ROS2actionServer:
    """
    A class abstracting a ROS 2 ActionServer
    """
    def __init__(self):
        # TODO: fillme
        raise NotImplementedError


class ROS2actionClient:
    """
    A class abstracting a ROS 2 ActionClient
    """
    def __init__(self):
        # TODO: fillme
        raise NotImplementedError
