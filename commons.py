class BaseScanner:

    def __init__(self, net_range, port, extended):
        self.net_range = net_range
        self.port = port
        self.extended = extended

    def load_from_file(self,filename):
        raise NotImplementedError

    def scan(self):
        raise NotImplementedError

    def print_results(self):
        raise NotImplementedError

    def write_to_file(self, out_file):
        raise NotImplementedError



class BaseHost:
    def __init__(self):
        self.address = ''
        self.port = ''
        self.nodes = []

class BaseNode:
    def __init__(self):
        self.name = ''
        self.address = ''
        self.port = ''

class Topic:
    def __init__(self):
        self.name = ''
        self.type = ''


class Service:
    def __init__(self):
        self.name = ''

class Parameter:
    def __init__(self):
        self.name = ''
        self.type = ''
        self.value = ''
