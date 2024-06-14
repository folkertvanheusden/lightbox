#! /usr/bin/python3

import random
import socket
import time

#UDP_IP = '192.168.65.150'
UDP_IP = '10.42.45.47'
UDP_PORT = 32001

while True:
    MESSAGE = bytearray()

    for d in range(0, 3):
        for x in range(0, 64, 8):
            for y in range(0, 8):
                b = random.getrandbits(8)
                MESSAGE.append(b)

    print(MESSAGE, len(MESSAGE))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

    time.sleep(0.1)
