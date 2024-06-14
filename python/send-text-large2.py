#! /usr/bin/python3

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import socket
import sys
import threading
import time

UDP_IP = '192.168.65.150'
UDP_PORT = 32000

text_canvas = None
text_draw = None
text_width = 0

#font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMono.ttf", 24)
font = ImageFont.truetype('/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf', 24)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def thrd():
    global text_canvas
    global text_draw
    global text_width
    global t

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 5020))

    while True:
        try:
            data, sender_addr = sock.recvfrom(1024)
            print('message from %r: %r' % (sender_addr, data))
            text = data.decode('utf-8')

            text_width, text_height = font.getsize(text)
            text_width += 64
            text_canvas = Image.new('1', (text_width, text_height))
            text_draw = ImageDraw.Draw(text_canvas)
            text_draw.text((0, 0), text, fill=(1), font=font)

            t = time.time()

        except Exception as e:
            print('exception: ', e)

th = threading.Thread(target=thrd)
th.daemon = True
th.start()

t = None

msgclr = bytearray()
for i in range(0, 192):
    msgclr.append(0)

while True:
    if text_draw:
        for outerx in range(0, text_width):
            start = time.time()
            img = Image.new('1', (64, 24))
            drw = ImageDraw.Draw(img)
            img.paste(text_canvas, (64 - outerx, 0))
            drw.rectangle([(0, 0), (63, 23)], outline=1)

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

            sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
            end = time.time()

            left = 0.025 - (end - start)
            if left > 0:
                time.sleep(left)

    if t:
        if time.time() - t >= 60:
            print('clear')
            t = None
            text_draw = None

            sock.sendto(msgclr, (UDP_IP, UDP_PORT))

            time.sleep(.2)
        elif not text_draw:
            print('clear now')
            t = None
            text_draw = None

    else:
        print('idle')
        time.sleep(1)
