#!/usr/bin/env python

import math
import sys
from datetime import *
import threading, queue

import pygame as pg
from pygame import gfxdraw
import serial
import struct



pg.init()

black = pg.Color("black")
white = pg.Color("white")
grey = (128, 128, 128)
red = pg.Color("red")

vmargin = 20

size = (1024, 768)

screen = pg.display.set_mode(size)
font = pg.font.Font(None, 32)

waiting = pg.font.Font(None, 64).render("WAITING FOR RESET", 0, white)
waiting_x = (1024 - waiting.get_width()) / 2
waiting_y = (768 - waiting.get_height()) / 2


class Timebase:
    DEFAULT_TIMEBASE_MS_IDX = 2 # 250 ms
    TIMEBASE_VALUES_MS = [50, 100, 250, 500, 1000]

    def __init__(self):
        self.idx = self.DEFAULT_TIMEBASE_MS_IDX
        self.update()

    def update(self):
        self.ms = self.TIMEBASE_VALUES_MS[self.idx]
        msg = "T: %d ms" % self.ms
        self.image = font.render(msg, 0, white)

    def increase(self):
        if self.idx < len(self.TIMEBASE_VALUES_MS) - 1:
            self.idx += 1
            self.update()

    def decrease(self):
        if self.idx > 0:
            self.idx -= 1
            self.update()


class DataCapture(threading.Thread):
    def __init__(self, tbase, q):
        threading.Thread.__init__(self, daemon=True)
        self.tbase = tbase
        self.queue = q
        self.ser = serial.Serial('/dev/ttyACM0', 115200)
        self.channel = 0

    def run(self):
        data = []
        reset = 0
        running = False
        idx = 0
        bs = bytearray(8)
        while True:
            b = self.ser.read()
            bs[idx] = b[0]

            if b[0] == 0xAA:
                reset += 1
                if reset == 8:
                    idx = 0
                    running = True
                    t0 = datetime.now()
                    continue
            else:
                reset = 0

            idx += 1
            idx %= 8

            if running and idx == 0:
                us = struct.unpack('<HHHH', bs)
                t = (datetime.now() - t0).total_seconds()
                data.append((t, us[self.channel]))
                if t * 1000 > self.tbase.ms:
                    t0 = datetime.now()
                    self.queue.put(data)
                    data = []


def adc_scale(x):
    return x / 4095.0 * 3.3

def make_points(tbase, signal):
    return [(1024 * 1000 * t / tbase.ms,
             768 - vmargin - adc_scale(y) / 3.3 * (768-2*vmargin))
            for t, y in signal if t <= tbase.ms]

def draw(tbase, signal):
    screen.fill(black)

    if signal is not None:
        pg.draw.line(screen, grey, (0,768-vmargin), (1024,768-vmargin))
        pg.draw.lines(screen, red, False, make_points(tbase, signal), 4)
        screen.blit(tbase.image, (50, 50))
    else:
        screen.blit(waiting, (waiting_x, waiting_y))



def main():
    tbase = Timebase()
    clock = pg.time.Clock()
    q = queue.SimpleQueue()

    DataCapture(tbase, q).start()

    signal = None
    while True:
        try:
            signal = q.get(False)
        except:
            pass

        draw(tbase, signal)

        for event in pg.event.get():
            if (event.type == pg.QUIT or
                (event.type == pg.KEYDOWN and event.key == pg.K_ESCAPE)):
                pg.quit()
                sys.exit()
            elif event.type == pg.KEYDOWN:
                if event.key == pg.K_t:
                    tbase.increase()
                elif event.key == pg.K_g:
                    tbase.decrease()

        pg.display.flip()
        clock.tick(40)

if __name__ == "__main__":
    main()
