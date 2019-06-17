

class ROS2Node:

    def __init__(self):
        self.name = ''
        self.domain_id = 0
        self.namespace = ''
        self.subscribed_topics = []
        self.published_topics = []


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




