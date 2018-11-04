#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio
import aiohttp
import logging

from aztarna.commons import BaseScanner

class IndustrialRouterScanner(BaseScanner):
    def __init__(self):
        super().__init__()

        self.timeout = aiohttp.ClientTimeout(total=3)
        self.hosts_eWON = []

        self.logger = logging.getLogger(__name__)

    async def scan_pipe(self):
        async for line in BaseScanner.stream_as_generator(asyncio.get_event_loop(), sys.stdin):
            str_line = (line.decode()).rstrip('\n')
            for port in self.ports:
                await self.scan_host(str_line, port)

    def scan_pipe_main(self):
        asyncio.get_event_loop().run_until_complete(self.scan_pipe())

    async def analyze_router(self, address, port):
        """
        Scan a router and gather all the information available.

        :param address: address of the industrial router
        :param port: port of the industrial router
        """
        async with aiohttp.ClientSession(loop=asyncio.get_event_loop(), timeout=self.timeout) as client:
            full_host = 'http://' + str(address) #+ ':' + str(port)
            try:
                async with client.get(full_host) as response:
                    server = response.headers.get('Server')
                    if server == 'eWON':
                        self.hosts_eWON.append(full_host)
            except Exception as e:
                self.logger.error('[-] Error connecting to host ' + str(full_host) + ': ' + str(e) + '\n')
            await client.close()

    async def scan_network(self):
        """
        Scan the provided network (from args) searching for industrial routers.
        """
        try:
            results = []
            for port in self.ports:
                for address in self.host_list:
                    results.append(self.analyze_router(address, port))

            for result in await asyncio.gather(*results):
                pass

        except ValueError as e:
            self.logger.error('Invalid address entered')
            raise e

    def print_results(self):
        """
        Print the information of all industrial routers detected.
        """
        for host in self.hosts_eWON:
            print("eWON router in " + host)

    def scan(self):
        """
        Run the scan for industrial routers hosts. Extended from :class:`aztarna.commons.BaseScanner`.
        This function is the one to be called externally in order to run the scans. Internally those scans are run
        with the help of asyncio.
        """
        asyncio.get_event_loop().run_until_complete(self.scan_network())
