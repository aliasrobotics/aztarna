import asyncio
from ipaddress import ip_network, IPv4Address, AddressValueError

from commons import BaseScanner, FileUtils
from sros.helpers import SROSNode, SROSHost, get_node_info, get_policies, get_sros_certificate


class SROSScanner(BaseScanner):

    def __init__(self):
        super().__init__()
        self.hosts = None
        self.addresses = []

    async def scan_host(self, address, master_port, timeout=1):
        sros_host = None
        master_address, port, master_cert = await get_sros_certificate(address, master_port, timeout)
        sem = asyncio.Semaphore(10)
        if master_cert:
            sros_host = SROSHost()
            sros_host.address = address
            master_node = get_node_info(master_cert)
            master_node.policies = get_policies(master_cert)
            sros_host.nodes.append(master_node)
            results = []
            if self.extended:
                for port in range(11310, 50000):
                    results.append(get_sros_certificate_sem(address, port, sem, timeout))

                for result in await asyncio.gather(*results):
                    print(result)
                    if result[2]:
                        node_info = get_node_info(result[2])
                        node_info.policies = get_policies(result[2])
                        sros_host.nodes.append(node_info)

        return sros_host

    async def scan_network(network_range, port):
        sros_hosts = []
        try:
            network = ip_network(network_range)
            if network.netmask == IPv4Address('255.255.255.255'):
                host_list = [IPv4Address(network_range)]
            else:
                host_list = list(network.hosts())
            for host_address in host_list:
                print('Scanning node {}'.format(host_address))
                nodes = await scan_host(host_address, port)
                if len(nodes) > 0:
                    sros_host = SROSHost()
                    sros_host.nodes = nodes
                    sros_hosts.append(sros_host)
        except AddressValueError:
            print('Invalid network entered')
        except Exception as e:
            print(e)

        return sros_hosts

    def scan(self):
        pass

    def print_results(self):
        for host in self.hosts:
            print(host.address)
            for node in host.nodes:
                print('\tNode name: {}'.format(node.name))
                print('\tPort: {}'.format(node.port))
                print('\tDemo CA Used: {}'.format(node.is_demo))
                print('\tPolicies:')
                for policy in node.policies:
                    print('\t\tType: {}'.format(policy.type))
                    print('\t\tPermission: {}'.format(policy.permissions))
                    print('\t\tValues: ')
                    for value in policy.values:
                        print('\t\t\t{}'.format(value))
                    print('')
                print('')

    def write_to_file(self, out_file):
        lines = []
        header = 'Address;Node;Port;Demo;Policy_Type;Value;Permission\n'
        lines.append(header)
        for host in self.hosts:
            for node in host.nodes:
                for policy in node.policies:
                    for value in policy.values:
                        line = '{};{};{};{};{};{};{}\n'.format(host.address, node.name, node.port, node.is_demo,
                                                               policy.type, value, policy.permissions)
                        lines.append(line)
        with open(out_file, 'w') as file:
            file.writelines(lines)

    def load_from_file(self, in_file):
        self.addresses = FileUtils.load_from_file(in_file)



