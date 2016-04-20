"""
 odometry thread
"""

from bbio.platform.util import sysfs
from bbio import *
from bbio.libraries.RotaryEncoder import RotaryEncoder
import math
import threading
import time

class Odometry(object):
    def __init__(self, tick_to_mm, rayon):
        self.encoderR = RotaryEncoder(RotaryEncoder.EQEP0)
        self.encoderL = RotaryEncoder(RotaryEncoder.EQEP2b)
        self.base_dir = self.encoderR.base_dir
        self.encoder_file = "%s/encoder" % self.base_dir
        self.X = 0.0
        self.Y = 0.0
        self.A = 0.0
        self.A_offset = 0.0
        self.tick_to_mm = tick_to_mm
        self.dist_to_mm = tick_to_mm / 2.0
        self.teta_to_radian = tick_to_mm / rayon
        self.previous = None
        
        self.lock = threading.RLock()
        self.myThread = None
        self.run  = True
        
        d,g, self.A = self.get_encoder() #flust 1st value
    
    def start(self):  
        if self.myThread != None:
            return
        self.myThread = threading.Thread(target=self.odo_thread)
        self.myThread.setDaemon(True)
        self.myThread.start()
    
    @property
    def position(self):
        with self.lock:
            return self.X, self.Y
        
    @position.setter
    def position(self, pos):
        with self.lock:
            self.X = pos[0]
            self.Y = pos[1]

    @property
    def angle(self):
        with self.lock:
            return self.A + self.A_offset

        
    @angle.setter
    def angle(self, angle):
        with self.lock:
            self.A = angle

    def get_encoder(self):
        enc = sysfs.kernelFileIO(self.encoder_file).split()
        d = int(enc[0])
        g = int(enc[1])
        angle = (d - g) * self.teta_to_radian

        ret = d - self.previous[0], g - self.previous[1], angle
        self.previous = (d, g)
        return ret
    
    def do_odo(self):
        dr, dl, angle = self.get_encoder()
        d_dist = (dr + dl) * self.dist_to_mm
        
        X = self.X + d_dist * math.cos(self.angle)
        Y = self.Y + d_dist * math.sin(self.angle)
        
        with self.lock:
            self.X = X
            self.Y = Y
            self.A = angle
    
    def test(self,nb):
        a = 0.0
        for i in range(nb):
            self.do_odo()
            a = a + self.angle
            print a
    
    
    def odo_thread(self):
        while self.run:
            self.do_odo()
            time.sleep(0.1)            
        
        
        
