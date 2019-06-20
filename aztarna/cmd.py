#!/usr/bin/env python
# -*- coding: utf-8 -*-
import asyncio
import logging
import re
from argparse import ArgumentParser
import argcomplete
import uvloop

from aztarna.ros.industrial.scanner import ROSIndustrialScanner
from aztarna.ros.ros2.scanner import ROS2Scanner
from aztarna.ros.sros import SROSScanner
from aztarna.ros.ros import ROSScanner
from aztarna.industrialrouters.scanner import IndustrialRouterAdapter

asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

logging.getLogger(__name__).setLevel(logging.DEBUG)

def main():
    """
    Main method
    """
    logging.basicConfig(level=logging.INFO, format="%(name)s - %(message)s")
    logger = logging.getLogger(__name__)
    parser = ArgumentParser(description='Aztarna')
    parser.add_argument('-t', '--type', help='<ROS/ros/SROS/sros/ROS2/ros2/IROUTERS/irouters> Scan ROS, SROS, ROS2 hosts or Industrial routers', required=True)
    parser.add_argument('-a', '--address', help='Single address or network range to scan.')
    parser.add_argument('-p', '--ports', help='Ports to scan (format: 13311 or 11111-11155 or 1,2,3,4)', default='11311')
    parser.add_argument('-i', '--input_file', help='Input file of addresses to use for scanning')
    parser.add_argument('-o', '--out_file', help='Output file for the results')
    parser.add_argument('-e', '--extended', help='Extended scan of the hosts', action='store_true')
    parser.add_argument('-r', '--rate', help='Maximum simultaneous network connections', default=100, type=int)
    parser.add_argument('--shodan', help='Use shodan for the scan types that support it.', action='store_true')
    parser.add_argument('--api-key', help='Shodan API Key')
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    try:
        if args.type == 'ROS' or args.type == 'ros':
            scanner = ROSScanner()
        elif args.type == 'SROS' or args.type == 'sros':
            scanner = SROSScanner()
        elif args.type == 'IROUTERS' or args.type == 'irouters':
            scanner = IndustrialRouterAdapter()
            if args.shodan is True:
                scanner.use_shodan = True
                scanner.shodan_api_key = args.api_key
                scanner.initialize_shodan()
        elif args.type.upper() == 'ROSIN':
            scanner = ROSIndustrialScanner()
        elif args.type.upper() == 'ROS2':
            scanner = ROS2Scanner()
        else:
            logger.critical('Invalid type selected')
            return
        if args.input_file:
            try:
                scanner.load_from_file(args.input_file)
            except FileNotFoundError:
                logger.critical('Input file not found')
        elif args.address:
            scanner.load_range(args.address)
        else:
            if args.type.upper() not in ['ROS2']:
                scanner.scan_pipe_main()
                return


        # TODO Implement a regex for port argument
        try:
                scanner.ports = range(int(args.ports.split('-')[0]), int(args.ports.split('-')[1]))
        except:
            try:
                scanner.ports = [int(port) for port in args.ports.split(',')]
            except:
                try:
                    scanner.ports.append(int(args.ports))
                except Exception as e:
                    logger.error('[-] Error: ' + str(e))

        scanner.extended = args.extended
        scanner.rate = args.rate
        scanner.scan()

        if args.out_file:
            scanner.write_to_file(args.out_file)
        else:
            scanner.print_results()
    except Exception as e:
        logger.critical('Exception occurred during execution')
        raise e


if __name__ == '__main__':
    main()
