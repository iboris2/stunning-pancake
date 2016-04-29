"""
 odometry thread
"""

from bbio.platform.util import sysfs
from bbio import *
from bbio.libraries.RotaryEncoder import RotaryEncoder
import math
import threading
import time
import os

class Odometry(object):
    REFRESH = 0.1
    def __init__(self, tick_to_mm, rayon, coef_d=-1,coef_g=1):
        self.coef_d = coef_d
        self.coef_g = coef_g
        self.encoderR = RotaryEncoder(RotaryEncoder.EQEP0)
        self.encoderL = RotaryEncoder(RotaryEncoder.EQEP2b)
        if os.system('insmod /root/bb/kernel/encoder/encoder.ko') == 0:
            print "encoder ok"
        self.base_dir = self.encoderR.base_dir
        self.encoder_file = "%s/encoder" % self.base_dir
        self.X = 0.0
        self.Y = 0.0
        self.A = 0.0
        self.A_offset = 0.0
        self.tick_to_mm = tick_to_mm
        self.dist_to_mm = tick_to_mm / 2.0
        self.teta_to_radian = tick_to_mm / (2.0 * math.pi * rayon)
        self.lock = threading.RLock()
        
        self.previous = None
        self.init_encoder()

        self.myThread = None
        self.run  = True
        
        self.start()
    
    def start(self):  
        if self.myThread != None:
            return
        self.myThread = threading.Thread(target=self.odo_thread)
        self.myThread.setDaemon(True)
        self.myThread.start()

    @property
    def encoder(self):
        with self.lock:
            return self.previous[0] * self.tick_to_mm, self.previous[1] * self.tick_to_mm
    
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
            self.A_offset = angle - self.A
    
    def init_encoder(self):
        enc = sysfs.kernelFileIO(self.encoder_file).split()
        d = int(enc[0]) * self.coef_d
        g = int(enc[1]) * self.coef_g
        angle = (d - g) * self.teta_to_radian
        with self.lock:
            self.previous = (d, g)
            self.A = angle

    def _get_encoder(self):
        enc = sysfs.kernelFileIO(self.encoder_file).split()
        d = int(enc[0]) * self.coef_d
        g = int(enc[1]) * self.coef_g
        angle = (d - g)

        ret = d - self.previous[0], g - self.previous[1], angle
        with self.lock:
            self.previous = (d, g)
        return ret
    
    def do_odo(self):
        dr, dl, angle = self._get_encoder()
        d_dist = (dr + dl) * self.dist_to_mm
        
        X = self.X + d_dist * math.cos(self.angle)
        Y = self.Y + d_dist * math.sin(self.angle)
        
        with self.lock:
            self.X = X
            self.Y = Y
            self.A = angle * self.teta_to_radian
    
    def test(self,nb):
        a = 0.0
        for i in range(nb):
            self.do_odo()
            a = a + self.angle
            print a
    
    
    def odo_thread(self):

        while self.run:
            self.do_odo()
            time.sleep(0.02)            
        
        
        
