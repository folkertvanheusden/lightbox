#! /usr/bin/python3

# (C) 2019-2024 by Folkert van Heusden <mail@vanheusden.com>

# This script requires the "opencv-python" package.

from lzjb import compress
import socket
import time

MC_IP = '226.1.1.9'
MC_PORT = 32009

from cv2 import *
# initialize the camera
cam = VideoCapture(0)   # 0 -> index of camera

def rescale_by_width(image, target_width, method=INTER_LANCZOS4):
    """Rescale `image` to `target_width` (preserving aspect ratio)."""
    h = int(round(target_width * image.shape[0] / image.shape[1]))
    return resize(image, (target_width, h), interpolation=method)

def set_pixel(im,x,y,new):
    im[y,x] = new

MULTICAST_TTL = 15

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, MULTICAST_TTL)

while True:
    sa, img = cam.read()

    if not sa:
        continue

    img = rescale_by_width(img, 64)

    avg = 0

    for y in range(0, 24):
        for x in range(0, 64):
            avg += img[y][x][0]
            avg += img[y][x][1]
            avg += img[y][x][2]

    avg /= 24 * 64 * 3
    print(avg)

    MESSAGE = bytearray()

    for d in range(0, 3):
        for x in range(0, 64, 8):
            for y in range(0, 8):
                yy = 7 - y + d  * 8

                b = 0
                for xx in range(0, 8):
                    b <<= 1

                    if (int(img[yy, x + 7 - xx][0]) + int(img[yy, x + 7 - xx][1]) + int(img[yy, x + 7 - xx][2])) / 3 >= avg:
                        b |= 1

                MESSAGE.append(b)

    #print(len(MESSAGE))

    time.sleep(0.1)

    data_lzjb = compress(MESSAGE)
    s.sendto(data_lzjb, (MC_IP, MC_PORT))
