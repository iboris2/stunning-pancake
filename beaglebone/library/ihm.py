#fork johnlr/raspberrypi-tm1637

import time
from bbio import *

"""
      A
     ---
  F |   | B
     -G-
  E |   | C
     ---
      D

"""


class TM1637:
    I2C_COMM1 = 0x40
    I2C_COMM2 = 0xC0
    I2C_COMM3 = 0x80
    digit_to_segment = [
        0b0111111, # 0
        0b0000110, # 1
        0b1011011, # 2
        0b1001111, # 3
        0b1100110, # 4
        0b1101101, # 5
        0b1111101, # 6
        0b0000111, # 7
        0b1111111, # 8
        0b1101111, # 9
        0b1110111, # A
        0b1111100, # b
        0b0111001, # C
        0b1011110, # d
        0b1111001, # E
        0b1110001  # F
        ]

    def __init__(self, clk, dio):
        self.clk = clk
        self.dio = dio
        self.brightness = 0x0f

        pinMode(self.clk, INPUT)
        pinMode(self.dio, INPUT)
        digitalWrite(self.clk, LOW)
        digitalWrite(self.dio, LOW)

    def bit_delay(self):
        return
   
    def set_segments(self, segments, pos=0):
        # Write COMM1
        self.start()
        self.write_byte(self.I2C_COMM1)
        self.stop()

        # Write COMM2 + first digit address
        self.start()
        self.write_byte(self.I2C_COMM2 + pos)

        for seg in segments:
            self.write_byte(seg)
        self.stop()

        # Write COMM3 + brightness
        self.start()
        self.write_byte(self.I2C_COMM3 + self.brightness)
        self.stop()

    def start(self):
        pinMode(self.dio, OUTPUT)
        self.bit_delay()
   
    def stop(self):
        pinMode(self.dio, OUTPUT)
        self.bit_delay()
        pinMode(self.clk, INPUT)
        self.bit_delay()
        pinMode(self.dio, INPUT)
        self.bit_delay()
  
    def write_byte(self, b):
      # 8 Data Bits
        for i in range(8):

            # CLK low
            pinMode(self.clk, OUTPUT)
            self.bit_delay()

            pinMode(self.dio, INPUT if b & 1 else OUTPUT)

            self.bit_delay()

            pinMode(self.clk, INPUT)
            self.bit_delay()
            b >>= 1
      
        pinMode(self.clk, OUTPUT)
        self.bit_delay()
        pinMode(self.clk, INPUT)
        self.bit_delay()
        pinMode(self.clk, OUTPUT)
        self.bit_delay()

        return

# def show_clock(tm):
#         t = localtime()
#         sleep(1 - time() % 1)
#         d0 = tm.digit_to_segment[t.tm_hour // 10] if t.tm_hour // 10 else 0
#         d1 = tm.digit_to_segment[t.tm_hour % 10]
#         d2 = tm.digit_to_segment[t.tm_min // 10]
#         d3 = tm.digit_to_segment[t.tm_min % 10]
#         tm.set_segments([d0, 0x80 + d1, d2, d3])
#         sleep(.5)
#         tm.set_segments([d0, d1, d2, d3])

class Display(TM1637):
    
    P = 0b1110011
    G = TM1637.digit_to_segment[6]
    CLEAR = 0
    
    def __init__(self, clk=GPIO2_2, dio=GPIO2_3):
        TM1637.__init__(self, clk, dio)
    
    def color(self, color=None):
        if color is None:
            seg = Display.CLEAR
        elif color == 'purple':
            seg = Display.P
        else:
            seg = Display.G
        self.set_segments([seg, 0])
    
    def hello(self):
        self.set_segments([0b1001, 0b1001,0b1001,0b1001])

    def starter(self,value):
        if value == 1:
            self.set_segments([0b1000],2)
        elif value == 2:
            self.set_segments([0b1001000],2)
        else:
            self.set_segments([0],2)
    def strategy(self, value):
        if value is None:
            self.set_segments([0],3)
        else:    
            self.set_segments([self.digit_to_segment[value]],3)

class Button():
    def __init__(self, gpio, pullup=None):
        self.gpio = gpio
        pull = -1
        if pullup is not None:
            pull = 1
        pinMode(self.gpio, INPUT, pull)
        self.prev_value = None
    
    @property
    def value(self):
        return digitalRead(self.gpio)
    
    def event(self):
        if self.prev_value == None:
            self.prev_value = self.value
            return None
        value = self.value
        if value != self.prev_value:
            self.prev_value = value
            print 'button event', self.gpio
            return value
        return None

class Ihm(object):
    IDLE = 0
    SELECT_COLOR = 1
    SELECT_STRATEGY = 2
    blink_delay = 1.0
    def __init__(self):
        self.prepare_lvl = 0
        self.display = Display(GPIO2_2, GPIO2_3)
        self.button  = Button(GPIO2_5,1)
        self.starter = Button(GPIO0_30)
    
        self.color = 'purple'
        self.strategy = 0
        self.time = time.time()
        self.display.hello()
        self.display.color(self.color)
        self.display.strategy(self.strategy)
        self.mode = Ihm.IDLE
    
    def do_events(self):
        t = time.time()
        if self.starter.value:
            self.display.starter(1)
        else:
            self.display.starter(0)
        if self.mode == Ihm.IDLE:
            if self.button.event() is not None:
                self.mode = Ihm.SELECT_COLOR
                self.time = t
            return
        if self.mode == Ihm.SELECT_COLOR:
            if self.button.event() is not None:
                self.mode = Ihm.SELECT_STRATEGY
                self.time = t
                return
            if t - self.time > Ihm.blink_delay:
                self.time = t
                self.display.color(None)
                time.sleep(0.1)
                if self.color == 'purple':
                    self.color = 'green'
                else:
                    self.color = 'purple'
                self.display.color(self.color)
            return
        if self.mode == Ihm.SELECT_STRATEGY:
            if self.button.event() is not None:
                self.mode = Ihm.IDLE
                self.time = t
                return
            if t - self.time > Ihm.blink_delay:
                self.time = t
                self.display.strategy(None)
                time.sleep(0.1)
                self.strategy = (self.strategy + 1) % 5
                self.display.strategy(self.strategy)
            return

    def prepare(self):
        #flush events
        self.button.event()
        while True:
            self.do_events()
            if self.starter.event() == LOW:
                self.display.color(self.color)
                self.display.strategy(self.strategy)
                self.prepare_lvl = 1
                return self.color, self.strategy
            time.sleep(0.1)

    def wait_starter(self):
        while True:
            if self.starter.value:
                self.display.starter(2)
            else:
                self.display.starter(0)
            if self.starter.event() == LOW:
                return True
            if self.button.event():
                return False


        

