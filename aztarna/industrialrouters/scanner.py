#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio
import aiohttp
import logging

from aztarna.commons import BaseScanner

import pkg_resources

class IndustrialRouterScanner(BaseScanner):
    def __init__(self):
        super().__init__()

        self.timeout = aiohttp.ClientTimeout(total=3)
        self.hosts_eWON = []
        self.hosts_eWON_secure = []
        self.hosts_moxa = []
        self.hosts_moxa_secure = []

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
            full_host = 'http://' + str(address) + ':' + str(port)
            try:
                async with client.get(full_host) as response:

                    async def fetch(client, host):
                        async with client.get(host) as resp:
                            assert resp.status == 200
                            return await resp.text()
                    server = response.headers.get('Server')
                    html = await fetch(client, full_host)
                    # print(html)
                    if "MoxaHttp" in server:
                            self.hosts_moxa.append(full_host)
                            return
            except Exception as e:
                self.logger.error('[-] Error connecting to Moxa host ' + str(full_host) + " " + str(e) + '\n')
            await client.close()
        # EWon
        auth = aiohttp.BasicAuth(login='adm', password='adm')
        async with aiohttp.ClientSession(auth=auth, loop=asyncio.get_event_loop(), timeout=self.timeout) as client:
            full_host = 'http://' + str(address) + ':' + str(port)
            try:
                async with client.get(full_host) as response:

                    async def fetch(client, host):
                        async with client.get(host) as resp:
                            assert resp.status == 200
                            return await resp.text()

                    server = response.headers.get('Server')
                    html = await fetch(client, full_host)
                    # print(html)
                    if server == 'eWON':
                        if response.status != 401:
                            filename = pkg_resources.resource_filename('aztarna', "industrialrouters/assets/eWON.html")
                            print(filename)
                            with open(filename, 'r') as content_file:
                                content = content_file.read()
                                content = content.replace("\t", "")
                                content = content.replace("\n", "")
                                content = content.replace(" ", "")
                                html = html.replace("\n", "")
                                html = html.replace("\t", "")
                                html = html.replace(" ", "")
                                if(html.find(content)==-1):
                                    self.hosts_eWON.append(full_host)
                                else:
                                    self.hosts_eWON_secure.append(full_host)
                        else:
                            self.hosts_eWON_secure.append(full_host)

            except Exception as e:
                self.logger.error('[-] Error connecting to eWON host ' + str(full_host) + " " + str(e) + '\n')
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
        for host in self.hosts_eWON_secure:
            print("eWON router in " + host + " is secure")
        for host in self.hosts_eWON:
            print("eWON router in " + host + " is not secure")
        for host in self.hosts_moxa_secure:
            print("Moxa router in " + host + " is secure")
        for host in self.hosts_moxa:
            print("MOXA router in " + host + " is not secure")


    def scan(self):
        """
        Run the scan for industrial routers hosts. Extended from :class:`aztarna.commons.BaseScanner`.
        This function is the one to be called externally in order to run the scans. Internally those scans are run
        with the help of asyncio.
        """
        asyncio.get_event_loop().run_until_complete(self.scan_network())
