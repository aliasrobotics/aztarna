#!/usr/bin/env python
# -*- coding: utf-8 -*-

import asyncio
from platform import system as system_name  # Returns the system/OS name
from subprocess import call as system_call  # Execute a shell command


class HelpersLINQ:

    @staticmethod
    def distinct(sequence):
        seen = set()
        for s in sequence:
            if not s in seen:
                seen.add(s)
                yield s


class HelpersROS:
    # Process response of type array = [ [a, [a1, a2] ] , [b, [b1, b2] ] ]
    @staticmethod
    def process_line(array_object):
        if len(array_object) == 0:
            return None

        topic_name = array_object[0]
        node_names = (HelpersLINQ.distinct(array_object[1]))

        return [topic_name, node_names]


class HelpersNetWorking:
    @staticmethod
    def ping(host):
        ret = None
        # Ping command count option as function of OS
        param = '-n' if system_name().lower()=='windows' else '-c'
        # Building the command. Ex: "ping -c 1 google.com"
        command = ['ping', param, '1', host]
        with open("/dev/null", "w+") as f:
            ret = system_call(command, stdout=f) == 0

        return ret


class FileUtils:
    @staticmethod
    def load_from_file(in_file):
        addresses = []
        with open(in_file, 'r') as file:
            for line in file.readlines():
                addresses.append(line.rstrip('\n'))
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