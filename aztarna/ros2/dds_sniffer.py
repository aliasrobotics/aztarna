import socket
from networking.ethernet import Ethernet
from networking.udp import UDP
from networking.pcap import Pcap
from networking.ipv4 import IPv4


TAB_1 = '\t - '
TAB_2 = '\t\t - '


def main():
    pcap = Pcap('capture.pcap')
    conn = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.ntohs(3))

    while True:
        raw_data, addr = conn.recvfrom(65535)
        pcap.write(raw_data)
        eth = Ethernet(raw_data)

        # IPv4
        if eth.proto == 8:
            ipv4 = IPv4(eth.data)

            # UDP
            if ipv4.proto == 17:
                udp = UDP(ipv4.data)
                print(TAB_1 + 'UDP Segment:')
                print(TAB_2 + 'Source Port: {}, Destination Port: {}, Length: {}'.format(udp.src_port, udp.dest_port, udp.size))
                print(udp.data)
                if udp.src_port == 7411:  # TODO: search for interested dds ports
                    conn.send(udp.data)

    pcap.close()


main()
