#! /usr/bin/python3

from lzjb import compress
import random
import socket
import time

MC_IP = '226.1.1.9'
MC_PORT = 32009
MC_TTL = 15

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MC_TTL)

while True:
    MESSAGE = bytearray()
    for d in range(0, 3):
        for x in range(0, 64, 8):
            for y in range(0, 8):
                b = random.getrandbits(8)
                MESSAGE.append(b)
    data_lzjb = compress(MESSAGE)

    s.sendto(data_lzjb, (MC_IP, MC_PORT))
    time.sleep(0.1)
