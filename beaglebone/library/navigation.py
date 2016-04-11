"""
 robot motion goto
"""
import math
import time

class MotorConfig:
    def __init__(self, navigation, acc=None, vmax=None):
        self.acc = acc
        self.acc_bkp = navigation.acc
        self.vmax = vmax
        self.vmax_bkp = navigation.max_speed
        self.navigation = navigation
        
    def __enter__(self):
        if self.acc is not None:
            self.navigation.acc = self.acc
        if self.vmax is not None:
            self.navigation.max_speed = self.vmax
            
    def __exit__(self, type, value, traceback):
        if self.acc is not None:
            self.navigation.acc = self.acc_bkp
        if self.vmax is not None:
            self.navigation.max_speed = self.vmax_bkp

class Navigation(object):
    def __init__(self, motors):
        #136mm * PI = 1600 step
        self.mm_to_step = 1600 / (math.pi * 136)
        self.step_to_mm = 1.0/self.mm_to_step
        self.motors = motors
        
        self.D = 305.0
        self.d = self.D/2.0
    
    @property
    def max_speed(self):
        s = self.motors.max_speed
        return tuple([self.step_to_mm * x for x in s])

    @max_speed.setter
    def max_speed(self, speed):
        print "n mspeed" + str(speed)
        if isinstance(speed,tuple):
            s = speed
        else:
            s = (speed, speed)
        self.motors.max_speed = tuple([self.mm_to_step * x for x in s])  

    @property
    def acc(self):
        a = self.motors.acc
        return tuple([self.step_to_mm * x for x in a])
    
    @acc.setter
    def acc(self, acc):
        if isinstance(acc,tuple):
            a = acc
        else:
            a = (acc, acc)
        self.motors.acc = tuple([self.mm_to_step * x for x in a]) 

    def move(self, mm):
        dist = mm * self.mm_to_step
        self.motors.move(dist, dist)

    def turn(self, angle, rayon=0):
        mA = angle * (rayon + self.d)
        mB = angle * (rayon - self.d)
        # compute speed and acc
        vmaxA, vmaxB = self.max_speed
        accA, accB = self.acc
        absA = abs(mA)
        absB = abs(mB)
        if absA > absB:
            coeff = absB/absA
            vmaxB = vmaxB * coeff
            accB = accB * coeff
        else:
            coeff = absA/absB
            vmaxA = vmaxA * coeff
            accA = accA * coeff
        
        with MotorConfig(self, (accA, accB), (vmaxA, vmaxB)):
            self.motors.move(mA * self.mm_to_step , mB * self.mm_to_step)
            self.wait()
    
    def wait(self):
        time.sleep(0.3)
        while True:
            rA, rB = self.motors.remaining()
            rA = abs(rA)
            rB = abs(rB)
            print rA, rB
            if rA <=1 and rB <= 1:
                break
            time.sleep(0.2)