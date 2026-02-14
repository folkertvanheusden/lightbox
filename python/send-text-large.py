#! /usr/bin/python3

# (C) 2019-2026 by Folkert van Heusden <folkert@komputilo.nl>

# This script requires the "python3-pil" and "lzjb" packages.

from lzjb import compress
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import socket
import sys
import threading
import time

MC_IP = '226.1.1.9'
MC_PORT = 32009
MC_TTL = 15

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MC_TTL)

text = 'Hello, world! Send a text packet to port 5001 to show it here. '

font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMono.ttf", 24)

def thrd():
    global text
    global t

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 5001))

    while True:
        try:
            data, sender_addr = sock.recvfrom(1024)
            print('message from %r: %r' % (sender_addr, data))
            text = data.decode('utf-8') + '  '

            t = time.time()

        except Exception as e:
            print('exception: ', e)

th = threading.Thread(target=thrd)
th.daemon = True
th.start()


t = time.time()

while True:
    if text:
        img = Image.new('1', (64, 24))
        drw = ImageDraw.Draw(img)
        drw.text((0, 0), text, fill=(1), font=font)
        drw.rectangle([(0, 0), (63, 23)], outline=1)
        text = text[1:] + text[0]
        data = img.getdata()

        MESSAGE = bytearray()

        for d in range(0, 3):
            for x in range(0, 64, 8):
                for y in range(0, 8):
                    yy = 7 - y + d * 8

                    xyo = yy * 64 + x

                    b = 0
                    for o in range(xyo + 7, xyo - 1, -1):
                        b <<= 1
                        b |= data[o]

                    MESSAGE.append(b)

        data_lzjb = compress(MESSAGE)
        s.sendto(data_lzjb, (MC_IP, MC_PORT))

    if t and time.time() - t >= 60:
        t = None
        text = None

        MESSAGE = bytearray()
        for i in range(0, 192):
            MESSAGE.append(0)

        data_lzjb = compress(MESSAGE)
        s.sendto(data_lzjb, (MC_IP, MC_PORT))

    time.sleep(.2)
