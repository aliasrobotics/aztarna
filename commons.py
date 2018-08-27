import asyncio


class BaseScanner:

    def __init__(self, net_range='', port=11311, extended=False):
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


class FileUtils:
    @staticmethod
    def load_from_file(in_file):
        addresses = []
        with open(in_file, 'r') as file:
            for line in file.readlines():
                addresses.append(line)
        return addresses

class PortScanner:

    @staticmethod
    async def check_port(ip, port):
        conn = asyncio.open_connection(ip, port)
        try:
            reader, writer = await asyncio.wait_for(conn, timeout=3)
            writer.close()
            return port
        except:
            return None

    @staticmethod
    async def check_port_sem(sem, ip, port):
        async with sem:
            return await PortScanner.check_port(ip, port)

    @staticmethod
    async def scan_host(address, start_port, end_port, max_conns=400):
        sem = asyncio.Semaphore(max_conns)
        ports = range(start_port, end_port)
        tasks = [asyncio.ensure_future(PortScanner.check_port_sem(sem, address, port)) for port in ports]
        responses = await asyncio.gather(*tasks)
        open_ports = list(filter(lambda x: x is not None, responses))
        return open_ports

