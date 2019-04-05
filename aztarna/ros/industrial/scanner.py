import asyncio
import logging
import sys
import traceback
from functools import reduce

import aiohttp
from aiohttp import ClientConnectorError
from aiohttp_xmlrpc.client import ServerProxy

from aztarna.commons import RobotAdapter
from aztarna.ros.ros.helpers import ROSHost


class ROSIndustrialScanner(RobotAdapter):

    def __init__(self, ports=[80], extended=False):
        RobotAdapter.__init__(self, ports, extended)
        self.rosin_nodes = ['/streaming_client',            # ABB
                            '/motion_download_interface',   # ABB
                            '/robot_state',                 # ABB
                            '/joint_trajectory_action',     # ABB
                            '/kuka_eki_hw_interface',       # KUKA
                            '/controller_spawner',          # KUKA
                            '/motion_streaming_interface',  # FANUC
                            '/industrial_robot_client',     # FANUC
                            '/joint_state',                 # FANUC
                            '/kuka_rsi_simulator'           # KUKA
                            ]
        self.timeout = aiohttp.ClientTimeout(total=3)
        self.logger = logging.getLogger(__name__)
        self.hosts = []
        self.rate = 1000

    def load_from_file(self, filename):
        RobotAdapter.load_from_file(self, filename)

    def load_range(self, net_range):
        RobotAdapter.load_range(self, net_range)

    @staticmethod
    def stream_as_generator(loop, stream):
        return RobotAdapter.stream_as_generator(loop, stream)

    def scan(self):
        asyncio.get_event_loop().run_until_complete(self.scan_network())

    def scan_pipe_main(self):
        asyncio.get_event_loop().run_until_complete(self.scan_pipe())

    def print_results(self):
        for host in self.hosts:
            self.logger.warning(f'[+] ROSIN Host Found at {host.address}:{host.port}!!!')

    def write_to_file(self, out_file):
        with open(out_file, 'w') as f:
            f.write('Address;Port;Node\n')
            for host in self.host_list:
                for node in host.nodes:
                    f.write(f'{host.address};{host.port};{node}\n')

    async def analyze_nodes(self, address, port):
        found_nodes = []
        async with self.semaphore:
            full_host = f'http://{address}:{port}'
            self.logger.info(f'[+] Scanning host at {full_host}')
            try:
                async with aiohttp.ClientSession(loop=asyncio.get_event_loop(), timeout=self.timeout) as client:
                    ros_master_client = ServerProxy(full_host, client=client)
                    code, msg, val = await ros_master_client.getSystemState('')
                    if code == 1:
                        nodes = list(map(lambda x: x[0], map(lambda x: x[1], reduce(lambda x, y: x + y, val))))
                        for node in nodes:
                            if node in self.rosin_nodes:
                                found_nodes.append(node)
                if len(found_nodes) > 0:
                    ros_host = ROSHost(address, port)
                    ros_host.nodes = found_nodes
                    self.hosts.append(ros_host)
            except ClientConnectorError:
                self.logger.debug(f'[-] Unable to connect to host {address}')
            except Exception as e:
                ex, msg, tb = sys.exc_info()
                traceback.print_tb(tb)
                self.logger.debug(f'[-] Connection error on host {address}')


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

    async def scan_pipe(self):
        async for line in RobotAdapter.stream_as_generator(asyncio.get_event_loop(), sys.stdin):
            str_line = (line.decode()).rstrip('\n')
            for port in self.ports:
                await self.analyze_nodes(str_line, port)









