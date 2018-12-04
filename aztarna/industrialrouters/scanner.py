import asyncio
from asyncio import ALL_COMPLETED
from typing import List

import aiohttp
from aiohttp import ClientTimeout
from colorama import Fore
from shodan.client import Shodan

from aztarna.commons import RobotAdapter


class BaseIndustrialRouter:
    def __init__(self):
        self.name = 'BaseRouter'
        self.address = None
        self.port = None
        self.valid_credentials = []
        self.alive = False
        self.protocol = None


class EWonRouter(BaseIndustrialRouter):
    def __init__(self):
        super(EWonRouter, self).__init__()
        self.name = 'EWon Router'


class MoxaRouter(BaseIndustrialRouter):
    def __init__(self):
        super(MoxaRouter, self).__init__()
        self.name = 'Moxa Router'


class WestermoRouter(BaseIndustrialRouter):
    def __init__(self):
        super(WestermoRouter, self).__init__()
        self.name = 'Westermo Router'


class BaseIndustrialRouterScanner:
    possible_headers = {}
    default_credentials = []
    router_cls = None
    url_path = ''

    def __init__(self):
        pass

    @classmethod
    def check_routers_shodan(cls, shodan: Shodan) -> List[BaseIndustrialRouter]:
        found_routers = []
        for field, values in cls.possible_headers.items():
            for value in values:
                for result in shodan.search_cursor('{}: {}'.format(field, value)):
                    router = cls.router_cls()
                    router.address = result['ip_str']
                    router.port = result['port']
                    router.protocol = result['_shodan']['module']
                    found_routers.append(router)
        return found_routers

    @classmethod
    async def check_is_router(cls, address: str, port: int) -> bool:
        async with aiohttp.ClientSession(timeout=ClientTimeout(0.5)) as client:
            uri = 'http://{}:{}'.format(address, port)
            async with client.get(uri) as response:
                for field, values in cls.possible_headers:
                    if response.headers.get(field) in values:
                        return True
                    else:
                        return False

    @classmethod
    async def check_default_password(cls, router):
        uri = '{}://{}:{}/{}'.format(router.protocol, router.address, router.port, cls.url_path)
        for user, password in cls.default_credentials:
            auth = aiohttp.BasicAuth(login=user, password=password)
            async with aiohttp.ClientSession(timeout=ClientTimeout(2)) as client:
                try:
                    async with client.request('GET', uri, auth=auth, ssl=False) as response:
                        if response.status == 200:
                            router.valid_credentials.append((user, password))
                        elif response.status == 401:
                            pass
                        router.alive = True
                except:
                    router.alive = False

    def check_router_credentials(self, routers: List[BaseIndustrialRouter]):
        async def check_router_credentials_aio(routers):
            futures = []
            for router in routers:
                futures.append(asyncio.ensure_future(self.check_default_password(router)))
            await asyncio.wait(futures, return_when=ALL_COMPLETED)

        asyncio.get_event_loop().run_until_complete(check_router_credentials_aio(routers))


class WestermoScanner(BaseIndustrialRouterScanner):
    possible_headers = {'Server': ['Westermo', 'EDW']}
    default_credentials = [('admin', 'westermo')]
    router_cls = WestermoRouter


class MoxaScanner(BaseIndustrialRouterScanner):
    possible_headers = {'Server': ['MoxaHttp', 'MoxaHttp/1.0', 'MoxaHttp/2.2']}
    default_credentials = [('admin', 'root'), ('', 'root'), ('', ''), ('admin', 'admin'), ('admin', '')]
    router_cls = MoxaRouter


class EWonScanner(BaseIndustrialRouterScanner):
    possible_headers = [{'Server': ['eWON']}]
    default_credentials = [('adm', 'adm',)]
    router_cls = EWonRouter
    url_path = 'Ast/MainAst.shtm'


class IndustrialRouterAdapter(RobotAdapter):
    router_scanner_types = [WestermoScanner, EWonScanner, MoxaScanner]

    def __init__(self):
        super().__init__()
        self.use_shodan = False
        self.shodan_api_key = None
        self.shodan_conn = None
        self.routers = []
        self.router_scanners = []
        for cls in IndustrialRouterAdapter.router_scanner_types:
            self.router_scanners.append(cls())

    def initialize_shodan(self):
        self.shodan_conn = Shodan(self.shodan_api_key)

    def scan_pipe_main(self):
        pass

    def print_results(self):
        for router in self.routers:
            print(Fore.Green + 'Name' + router.name + Fore.RESET)
            print('\tAddress: {}:{}'.format(router.address, router.port))
            print('\tProtocol: ' + router.protocol)
            if router.alive:
                print('\t' + Fore.GREEN + 'Alive' + Fore.RESET)
                if len(router.valid_credentials):
                    print('\t' + Fore.RED + 'Found credentials:' + Fore.RESET)
                    for user, password in router.valid_credentials:
                        print('\t\tUsername: {} Password: {}'.format(user, password))
            else:
                print('\t' + Fore.RED + 'Unreachable' + Fore.RESET)

    def write_to_file(self, out_file):
        header = 'Type;Address;Port;Protocol;Valid Credentials\n'
        with open(out_file, 'w') as file:
            file.write(header)
            for router in self.routers:
                line = '{};{};{};{};{}\n'.format(router.name, router.address, router.port,
                                                 router.protocol, router.valid_credentials)
                file.write(line)

    def scan(self):
        if self.use_shodan:
            self.initialize_shodan()
            for scanner in self.router_scanners:
                self.routers.append(scanner.check_routers_shodan(self.shodan_conn))
