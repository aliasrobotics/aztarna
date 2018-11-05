#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio
from platform import system as system_name  # Returns the system/OS name
from subprocess import call as system_call  # Execute a shell command

class HelpersLINQ:
    """
    A helper class for emulating .NET useful methods.
    """
    @staticmethod
    def distinct(sequence):
        seen = set()
        for s in sequence:
            if not s in seen:
                seen.add(s)
                yield s

class HelpersNetWorking:
    """
    A helper class that checks networking related data
    """
    @staticmethod
    def ping(host):
        """
        A method that replicates the command line ping utility.

        :param host: Host to ping to
        :return: A boolean type that means if the ping reaches the destination or not
        """
        ret = None
        # Ping command count option as function of OS
        param = '-n' if system_name().lower()=='windows' else '-c'
        # Building the command. Ex: "ping -c 1 google.com"
        command = ['ping', param, '1', host]
        with open("/dev/null", "w+") as f:
            ret = system_call(command, stdout=f) == 0

        return ret

class PortScanner:
    """
    A base class that provides methods to check correct por scans.
    """
    @staticmethod
    async def check_port(ip, port):
        """
        Checks if a certain port is open
        :param ip: The host's IP address
        :param port: The host's port
        :return: The scanned port if open, else None
        """
        conn = asyncio.open_connection(ip, port)
        try:
            reader, writer = await asyncio.wait_for(conn, timeout=3)
            writer.close()
            return port
        except:
            return None

    @staticmethod
    async def check_port_sem(sem, ip, port):
        """
        Calls to :method:check_port with a Semaphore to avoid too many open connections.

        :param sem:
        :param ip:
        :param port:
        :return:
        """
        async with sem:
            return await PortScanner.check_port(ip, port)

    @staticmethod
    async def scan_host(address, start_port, end_port, max_conns=400):
        """

        :param address: IPv4 address to scan
        :param start_port: First port value to scan
        :param end_port: Last port value to scan
        :param max_conns: Maximum simultaneous number of connections
        :return:
        """
        sem = asyncio.Semaphore(max_conns)
        ports = range(start_port, end_port)
        tasks = [asyncio.ensure_future(PortScanner.check_port_sem(sem, address, port)) for port in ports]
        responses = await asyncio.gather(*tasks)
        open_ports = list(filter(lambda x: x is not None, responses))

        return open_ports
