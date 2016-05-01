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
    n.move(20)
n.motors.disable()
g.calibration()
ihm.wait_starter()
g.clamp(210, 0)
#take sand
sand = Vector(play.sand[0])
sand += play.vectorColor((0, robot.dist_front + 30))
n.goto(sand)

n.cap(play.capColor(-math.pi/2))
with MotorConfig(n, 150, 150):
    n.move(100)
    g.clamp(135,0)
#with MotorConfig(n, 400, 150):
    n.goto(Vector(play.build_area[1]) + play.vectorColor((-200,280 + robot.dist_front)))
    #g.addJob(lambda: g.clamp(134,play.clampColor(-63)))
    g.clamp(130,play.clampColor(60))
    g.quiet()
    n.cap(play.capColor(-math.pi/2))
with MotorConfig(n, 300, 220):
    n.move_contact(0, 195)
    n.move(-2)
    #release block
    g.clamp(130+60,play.clampColor(60-5))
    g.up(80)
    #take cylindre
    g.clamp(61)
    h = 90
    g.up(h)
    g.clamp(61, play.clampColor(-80),h)
    #depose tower
    g.up(15)
    #release
    g.clamp(100)
    n.move(-115)
    g.up(20)
    #open prepare tqcke 2 block
    g.clamp(138+40,play.clampColor(60-5))
    n.move(65)
    #take 2 block
    g.clamp(120,play.clampColor(60-5),20)
    g.up(28)
    n.move(-50)
    g.clamp(120,play.clampColor(-44))
    n.move(55)
    g.up(15)
    g.clamp(182)
    n.move(-150)
    n.motors.disable()
