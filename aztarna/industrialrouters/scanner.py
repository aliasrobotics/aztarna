import asyncio
import logging
import random
import re
import ssl
from asyncio import ALL_COMPLETED, Semaphore
from asyncio import TimeoutError
from typing import List

import aiohttp
from aiohttp import ClientTimeout
from colorama import Fore
from ipwhois import IPWhois
from shodan.client import Shodan

from aztarna.commons import RobotAdapter

logger = logging.getLogger(__name__)


class BaseIndustrialRouter:
    def __init__(self):
        self.name = 'BaseRouter'
        self.address = None
        self.port = None
        self.valid_credentials = []
        self.alive = False
        self.protocol = None
        self.country = ''
        self.asn_description = ''


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


class SierraRouter(BaseIndustrialRouter):
    def __init__(self):
        super(SierraRouter, self).__init__()
        self.name = 'Sierra Wireless Router'


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
                    if result['_shodan']['module'] == 'http-simple-new':
                        router.protocol = 'http'
                    elif result['_shodan']['module'] == 'https-simple-new':
                        router.protocol = 'https'
                    else:
                        router.protocol = result['_shodan']['module']

                    router.protocol = result['_shodan']['module']
                    found_routers.append(router)
        logger.info('[+] Shodan found {} routers. Scanner: {}'.format(len(found_routers), cls.__name__))
        return found_routers

    @classmethod
    async def check_is_router(cls, address: str, port: int, semaphore=Semaphore()) -> BaseIndustrialRouter:
        context = ssl.create_default_context()
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        context.options &= ~ssl.OP_NO_SSLv3
        async with semaphore:
            async with aiohttp.ClientSession(timeout=ClientTimeout(2)) as client:
                uri = 'http://{}:{}'.format(address, port)
                print('[+] Connecting to {}'.format(address))
                async with client.get(uri, ssl=context) as response:
                    for field, values in cls.possible_headers:
                        if response.headers.get(field) in values:
                            router = cls.router_cls()
                            router.address = address
                            router.port = port
                            return router
                        else:
                            return None

    def check_routers(self, addresses, ports):
        async def check_routers_aio(addresses, ports):
            semaphore = Semaphore(100)
            futures = []
            routers = []
            for address in addresses:
                for port in ports:
                    futures.append(asyncio.ensure_future(self.check_is_router(address, port, semaphore=semaphore)))
            done, pending = await asyncio.wait(futures)
            for future in done:
                if future:
                    routers.append(future)
            return routers

        return asyncio.run(check_routers_aio(addresses, ports), debug=True)

    @classmethod
    async def check_default_password(cls, router, semaphore=Semaphore()):
        uri = '{}://{}:{}/{}'.format(router.protocol, router.address, router.port, cls.url_path)
        context = ssl.create_default_context()
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        context.options &= ~ssl.OP_NO_SSLv3
        async with semaphore:
            for user, password in cls.default_credentials:
                auth = aiohttp.BasicAuth(login=user, password=password)
                async with aiohttp.ClientSession(timeout=ClientTimeout(20), auth=auth) as client:
                    try:
                        print('[+] Connecting to {}:{}'.format(router.address, router.port))
                        async with client.request('GET', uri, ssl=context) as response:
                            if response.status == 200:
                                router.valid_credentials.append((user, password))
                                logger.info(
                                    '[+] Default credential worked for router {}:{}'.format(router.address,
                                                                                            router.port))
                            elif response.status == 401:
                                logger.info(
                                    '[-] Default credential did not work for router {}:{}'.format(router.address,
                                                                                                  router.port))
                            router.alive = True
                    except TimeoutError:
                        router.alive = False
                        logger.warning(
                            '[-] Unsuccessful connection to router {}:{}'.format(router.address, router.port))
                    except ConnectionRefusedError:
                        logger.warning(
                            '[-] Unsuccessful connection to router {}:{}'.format(router.address, router.port))
                        router.alive = True
                    except Exception:
                        logger.exception(
                            '[-] Unsuccessful connection to router {}:{}'.format(router.address, router.port))
                        router.alive = False

    def check_router_credentials(self, routers: List[BaseIndustrialRouter]):
        async def check_router_credentials_aio(routers):
            semaphore = Semaphore(10)
            futures = []
            for router in routers:
                if isinstance(router, self.__class__.router_cls):
                    futures.append(asyncio.ensure_future(self.check_default_password(router, semaphore=semaphore)))

            await asyncio.wait(futures, return_when=ALL_COMPLETED, )

        asyncio.run(check_router_credentials_aio(routers), debug=True)

    def get_address_info(self, routers):
        for router in routers:
            if isinstance(router, self.__class__.router_cls):
                try:
                    whois = IPWhois(router.address)
                    results = whois.lookup_rdap(depth=1)
                    if results['asn_country_code']:
                        router.country = results['asn_country_code']
                    if results['asn_description']:
                        router.asn_description = results['asn_description']
                except:
                    pass


class WestermoScanner(BaseIndustrialRouterScanner):
    possible_headers = {'Server': ['Westermo', 'EDW']}
    default_credentials = [('admin', 'westermo')]
    router_cls = WestermoRouter


class MoxaScanner(BaseIndustrialRouterScanner):
    possible_headers = {'Server': ['MoxaHttp', 'MoxaHttp/1.0', 'MoxaHttp/2.2']}
    default_credentials_http1 = [('root', 'efa59ad49b7bc93a9a7bb1004f24b1cc'),  # Root
                                 ('', 'd41d8cd98f00b204e9800998ecf8427e'),  # Empty
                                 ('admin', 'd8a1dd02029af4e10b495bc3ab03859e')]  # Admin

    default_credentials_http2 = [('admin', 'root', '63a9f0ea7bb98050796b649e85481845'),  # Root
                                 ('', 'root', '63a9f0ea7bb98050796b649e85481845'),  # Root
                                 ('admin', 'admin', '21232f297a57a5a743894a0e4a801fc3'),  # Admin
                                 ('admin', '', 'd41d8cd98f00b204e9800998ecf8427e')]  # Empty
    valid_login_text_moxahttp_2_2 = 'FRAME name="main" src="main.htm"'
    valid_login_text_moxahttp_1_0 = 'FRAME name=main src=main.htm'
    router_cls = MoxaRouter

    @classmethod
    def get_challenge_moxahttp_1_0(cls, text):
        regexp = 'set\(\"FakeChallenge\",\"(?P<challenge>[A-Z0-9]+)\"\)\;'
        match = re.search(regexp, text.rstrip())
        if match:
            try:
                return match.group('challenge')
            except IndexError:
                return None
        else:
            return None

    @classmethod
    def get_challenge_moxahttp_2_2(cls, text):
        regexp = '<INPUT type=hidden name=FakeChallenge value=(?P<challenge>[A-Z0-9]+)>'
        match = re.search(regexp, text)
        if match:
            try:
                return match.group('challenge')
            except IndexError:
                return None
        else:
            return None

    @classmethod
    async def check_default_password(cls, router, semaphore=Semaphore()):
        uri = '{}://{}:{}/{}'.format(router.protocol, router.address, router.port, cls.url_path)
        context = ssl.create_default_context()
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        context.options &= ~ssl.OP_NO_SSLv3
        context.set_ciphers('HIGH:!DH:!aNULL')
        async with semaphore:
            async with aiohttp.ClientSession(timeout=ClientTimeout(20)) as client:
                async with client.request('GET', uri, ssl=context) as response:
                    router.alive = True
                    content = str(await response.content.read())
                    if cls.valid_login_text_moxahttp_2_2 in content or cls.valid_login_text_moxahttp_1_0 in content:
                        router.valid_credentials.append(('admin', 'no password'))
                    else:
                        if response.headers.get('Server') == 'MoxaHttp/1.0':
                            await cls.check_password_moxahttp_1_0(client, context, content, router)
                        elif response.headers.get('Server') == 'MoxaHttp/2.2':
                            await cls.check_password_moxahttp_2_2(client, context, content, router)

    @classmethod
    async def check_password_moxahttp_1_0(cls, client, context, content, router):
        challenge = cls.get_challenge_moxahttp_1_0(content)
        for clear_password, password in cls.default_credentials_http1:
            uri = '{}://{}:{}/home.htm?Password={}&Submit=Submit&token_text=&FakeChallenge={}' \
                .format(router.protocol, router.address, router.port, password, challenge)
            async with client.request('GET', uri, ssl=context) as response:
                content = str(await response.content.read())
                if cls.valid_login_text_moxahttp_2_2 in content:
                    router.valid_credentials.append(clear_password)

    @classmethod
    async def check_password_moxahttp_2_2(cls, client, context, content, router):
        uri = '{}://{}:{}/'.format(router.protocol, router.address, router.port)
        challenge = cls.get_challenge_moxahttp_2_2(content)
        for user, clear_password, password in cls.default_credentials_http2:
            payload = {
                'Username': user,
                'MD5Password': password,
                'Submit.x': random.randint(0, 50),
                'Submit.y': random.randint(0, 50)
            }
            if challenge:
                payload['FakeChallenge'] = challenge
            async with client.post(uri, data=payload, ssl=context) as response:
                content = str(await response.content.read())
                if cls.valid_login_text_moxahttp_2_2 in content:
                    router.valid_credentials.append((user, clear_password))


class EWonScanner(BaseIndustrialRouterScanner):
    possible_headers = {'Server': ['eWON']}
    default_credentials = [('adm', 'adm',)]
    router_cls = EWonRouter
    url_path = 'Ast/MainAst.shtm'


class SierraWirelessScanner(BaseIndustrialRouterScanner):
    possible_headers = {'Server': ['Sierra Wireless Inc, Embedded Server']}
    default_credentials = [('sconsole', '12345'),
                           ('', 'admin'),
                           ('', 'swiadmin'),
                           ('sconsole', '12345'),
                           ('user', '12345'),
                           ('viewer', '12345'),
                           ('admin', '')]
    failed_message = 'Invalid UserName / Password'
    router_cls = SierraRouter

    @classmethod
    async def check_default_password(cls, router, semaphore=Semaphore()):
        url = '{}://{}:{}/xml/Connect.xml'.format(router.protocol, router.address, router.port)
        headers = {'Accept': 'application/xml, text/xml, */*; q=0.01',
                   'Accept-Encoding': 'gzip, deflate',
                   'Content-Type': 'text/xml',
                   'X-Requested-With': 'XMLHttpRequest'}
        context = ssl.create_default_context()
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
        context.options &= ~ssl.OP_NO_SSLv3
        context.set_ciphers('HIGH:!DH:!aNULL')
        async with aiohttp.ClientSession(timeout=ClientTimeout(20), headers=headers) as client:
            for user, password in cls.default_credentials:
                payload = '''<request xmlns="urn:acemanager">
<connect>
<login>{}</login>
<password><![CDATA[{}]]></password>
</connect>
</request>
                '''.format(user, password)
                async with client.post(url, data=bytes(payload, 'utf-8')) as response:
                    content = str(await response.content.read())
                    if not cls.failed_message in content:
                        router.valid_credentials.append((user, password))


class IndustrialRouterAdapter(RobotAdapter):
    router_scanner_types = [WestermoScanner, EWonScanner, MoxaScanner, SierraWirelessScanner]

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

    def scan_network(self):
        for scanner in self.router_scanners:
            self.routers = scanner.check_routers(self.host_list, self.ports)
            scanner.check_router_credentials(self.routers)

    def print_results(self):
        for router in self.routers:
            print(Fore.Green + 'Name' + router.name + Fore.RESET)
            print('\tAddress: {}:{}'.format(router.address, router.port))
            print('\tProtocol: ' + router.protocol)
            print('\tCountry: ' + router.country)
            print('\tASN Description: ' + router.asn_description)
            if router.alive:
                print('\t' + Fore.GREEN + 'Alive' + Fore.RESET)
                if len(router.valid_credentials):
                    print('\t' + Fore.RED + 'Found credentials:' + Fore.RESET)
                    for user, password in router.valid_credentials:
                        print('\t\tUsername: {} Password: {}'.format(user, password))
            else:
                print('\t' + Fore.RED + 'Unreachable' + Fore.RESET)

    def write_to_file(self, out_file):
        header = 'Type;Address;Port;Protocol;Alive;Country;ASN Description;Valid Credentials\n'
        with open(out_file, 'w') as file:
            file.write(header)
            for router in self.routers:
                line = '{};{};{};{};{};{};{};{}\n'.format(router.name, router.address, router.port,
                                                          router.protocol, router.alive, router.country,
                                                          router.asn_description, router.valid_credentials)
                file.write(line)

    def scan(self):
        if self.use_shodan:
            self.initialize_shodan()
            for scanner in self.router_scanners:
                self.routers = self.routers + scanner.check_routers_shodan(self.shodan_conn)
                scanner.check_router_credentials(self.routers)
                # scanner.get_address_info(self.routers)
        else:
            for scanner in self.router_scanners:
                for address in self.host_list:
                    for port in self.ports:
                        self.routers.append(scanner.check_routers(address, port))
                        scanner.check_router_credentials(self.routers)
                # scanner.get_address_info(self.routers)
