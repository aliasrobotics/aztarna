from commons import BaseNode, BaseTopic


class Node(BaseNode):
    def __init__(self, name):
        super().__init__()

        self._published_topics = []
        self._subscribed_topics = []
        self._services = []


class Topic(BaseTopic):
    def __init__(self, name, topic_type):
        super().__init__()
        self.name = name
        self.type = topic_type

    def __str__(self):
        return self.name + '(Type: ' + self.type + ')'
