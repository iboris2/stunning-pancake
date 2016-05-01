import sys, time
sys.path.append('library')
from playground import *
from ihm import *
from navigation import *
import motor
import evitement
import bbio
import gobgob
import actuator
import robot
import math
from vector import *

class myi2c(object):
    def __init__(self, i2c):
        self.i2c = i2c
    
    def write(self,a,b):
        nbtry = 3
        while nbtry:
            nbtry = nbtry - 1
            try:
                ret = self.i2c.write(a, b)
                return ret
            except:
                print "i2c ERROR"
                pass
        
    
    def readTransaction(self,a,b,c):
        nbtry = 3
        while nbtry:
            nbtry = nbtry - 1
            try:
                ret = self.i2c.readTransaction(a,b,c)
                return ret
            except:
                print "i2c ERROR"
                pass

_s = bbio.Wire1
_s2 = bbio.Wire2

_s.begin()
_s2.begin()

s = myi2c(_s)
s2 = myi2c(_s2)

g = gobgob.Gobgob(s2)
n = Navigation(motor.StepperBlock(s),evitement.Obstacle(s2)) 
a = actuator.Actuator(s)
n.motors.disable()
ihm = Ihm()
color, strategy = ihm.prepare()
play = Playground(color)
n.position = play.start_pos
n.angle = play.start_angle
print "start pos", n.position, n.angle
with MotorConfig(n, 200, 150):
    n.move(10)
n.motors.disable()
g.calibration()
ihm.wait_starter()
g.clamp(210, 0)
#take sand
#sand = Vector(play.sand[0])
#sand += play.vectorColor((0, robot.dist_front + 30))
#n.goto(sand)

n.cap(play.capColor(-math.pi/2))
time.sleep(2)
with MotorConfig(n, 150, 150):
    n.move(100)
    g.clamp(135,0)
    time.sleep(1)
#with MotorConfig(n, 400, 150):
    #n.goto(Vector(play.build_area[1]) + play.vectorColor(-200-300,160))
    #g.addJob(lambda: g.clamp(134,play.clampColor(-63)))
    g.clamp(134,play.clampColor(-63))
    time.sleep(0.5)
    g.quiet()
    time.sleep(0.5)
    n.cap(-math.pi/2)
with MotorConfig(n, 200, 150):
    n.move_contact( 0, 160)
    n.move(-2)
    g.clamp(134+40,play.clampColor(-63+5))
    g.up(80)
    g.clamp(62)
    g.up(90)
    g.clamp(62, play.clampColor(80))
    g.up(15)
    g.clamp(100)
    time.sleep(0.2)
    n.move(-115)
    g.clamp(138+40,play.clampColor(-63+5))
    time.sleep(0.2)
    n.move(65)
    g.clamp(134)
    time.sleep(0.2)
    n.move(-50)
    g.clamp(134,play.clampColor(44))
    time.sleep(0.2)
    n.move(55)
    g.clamp(180)
    time.sleep(0.2)
    n.move(-150)
