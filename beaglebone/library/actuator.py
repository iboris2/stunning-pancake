from servo import *
import time

class Poisson(object):
    CLOSE = -84
    UP = -45
    DOWN = 68
    RELEASE = 64
    def __init__(self, servo):
        self.servo = servo
        self.servo.angle_init(Poisson.CLOSE)
 
    def close(self):
        self.servo.angle = Poisson.CLOSE
        self.servo.angle = Poisson.CLOSE     

    def up(self):
        self.servo.move(Poisson.UP)
        self.servo.angle = Poisson.UP
        
    def speedUp(self):
        self.servo.angle = Poisson.UP
        self.servo.angle = Poisson.UP
    
    def down(self):
        self.servo.angle = Poisson.DOWN
        self.servo.angle = Poisson.DOWN

    def release(self):
        self.servo.move(Poisson.DOWN, 195,270)
        self.servo.angle = Poisson.DOWN
        
    def disable(self):
        self.servo.disable()

class Umbrella(object):
    OPEN = -28
    CLOSE = 26
    def __init__(self, servo):
        self.servo = servo
        self.servo.angle_init(Umbrella.CLOSE)
        self.servo.disable()
    
    def open(self):
        print "umbrella"
        time.sleep(0.1)
        self.servo.angle = Umbrella.OPEN
        self.servo.angle = Umbrella.OPEN
    
    def disable(self):
        self.servo.disable()

class Actuator(object):
    def __init__(self, i2c):
        self.controler = ServoController(i2c)
        self.poisson = Poisson(self.controler.servos[0])
        self.umbrella = Umbrella(self.controler.servos[1])