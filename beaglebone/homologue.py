import rlcompleter, readline
readline.parse_and_bind('tab:complete')


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
        nbtry = 4
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

def prepare_small():
    prepare_tower(0)
    
def prepare_big():
    prepare_tower(1)
       
def prepare_tower(big):
    with MotorConfig(n, 400, 220):
        if big:
            offset_h = 60
            side = -1
        else:
            offset_h = 0
            side = 1        
        g.clamp(61)
        g.up(90+offset_h)
        g.clamp(61, play.clampColor(-95)*side)
        g.up(10)
        g.clamp(e=103,H=15)
        g.up(20)
        n.move(-130)
        prev = 100
        g.clamp(prev + 180, play.clampColor(-95 +180/2.0)*side)
        with MotorConfig(n, 280, 220):
            dist = 102+30
            if big:
                dist += 35
            n.move(dist)
            g.clamp(185,0)

def deposeBig():
    with MotorConfig(n, 400, 220):
        g.clamp(120, play.clampColor(-55))
        #release block
        g.clamp(120+70, play.clampColor(-55))
        #take tower
        g.up(80+60)
        g.clamp(61)
        h = 90+60
        g.up(h)
        g.clamp(61, play.clampColor(80),h)
        n.move(-50)
        #depose tower
        g.up(10)
        #release
        g.clamp(100)
        n.move(-125+50)
        g.clamp(120+95, play.clampColor(-55),80)
        g.up(80)
        n.move(70)
        #take 2 block
        g.clamp(120,play.clampColor(-55))
        g.up(90)
        n.move(-90)
        g.up(20)
        g.clamp(120,play.clampColor(20),20)
        n.move(38)
        g.up(10)
        g.clamp(182)

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
print "ma position"
print n.position
n.blockage = Blockage.NONE

g.clamp(150, play.clampColor(38),85)
#g.up(80)
#g.clamp(150, play.clampColor(-38),85+60)
sand = Vector(play.sand[0])
sand += play.vectorColor((30, robot.dist_front + 35))
n.goto(sand)
print "ma position"
print n.position
n.cap(play.capColor(-math.pi/2))

with MotorConfig(n, 270, 220):
    n.move(100+20)
prepare_small()
dist_bord = 280
with MotorConfig(n, 300, 220):
    n.goto(Vector(play.build_area[1]) + play.vectorColor((-200,dist_bord + robot.dist_block)),robot.rot_rayon)   
    n.cap(play.capColor(-math.pi/2),robot.rot_rayon)

    n.move_contact(0, dist_bord-30)
    g.clamp(240)
    n.move(-750)
    
#close hut
n.reculeto(play.close_hut)
n.cap(0)
with MotorConfig(n, 300, 220): 
    n.move_contact(0, -90)
    n.cap(0)
    n.move_contact(0, -50)
n.move(350)

#goto next block
n.goto(Vector(play.sand[1])+Vector(420,0))
n.cap(-math.pi)
x,y = n.position
g.clamp(292,play.clampColor(-15),55)
with MotorConfig(n, 200, 220): 
    n.move_contact(0, x - robot.dist_block + 20)
    n.move(-2)
    g.up(20)
    prev = 292
    suiv = 120
    g.clamp(suiv, play.clampColor(-15 +(suiv-prev)/2.0), 20)
    n.move(-200)

n.motors.disable()
g.disable()
toto()
deposeBig()
with ObstacleConfig(n, blockage = Blockage.NONE):
    with MotorConfig(n, 400, 220):
        n.move(100)
        n.goto(play.vectorColor((520,850)))
        g.up(60)
        g.clamp(230, play.clampColor(-20))
        n.goto(play.vectorColor((460,700)))
        n.cap(-math.pi)
        n.move_contact(0, 460 - robot.dist_block)
        #take clock
        n.move(-2)
        g.up(20)
        g.clamp(110, play.clampColor(-80))
        g.up(25)
    with MotorConfig(n, 400, 220):
        n.move(-300)
        n.goto(play.vectorColor((520,850)))
        n.goto(play.vectorColor((1050,850)))
        n.goto(play.vectorColor((1050,850)))
        n.cap(play.capColor(-math.pi/2))
        n.move_contact(0, abs(n.position[1]) - 24 - robot.dist_block -60 +20)
        #n.goto(Vector(play.build_area[1]) + play.vectorColor((-200,60 + 280 + robot.dist_front)))
        g.clamp(115, play.clampColor(-55))
        #release block
        g.clamp(115+70, play.clampColor(-55))
        #take tower
        g.up(80+60)
        g.clamp(61)
        h = 90+60
        g.up(h)
        g.clamp(61, play.clampColor(80),h)
        n.move(-50)
        #depose tower
        g.up(10)
        #release
        g.clamp(100)
        n.move(-125+50)
        g.clamp(115+95, play.clampColor(-55),80)
        g.up(80)
        n.move(70)
        #take 2 block
        g.clamp(120,play.clampColor(-55))
        g.up(90)
        n.move(-90)
        g.up(20)
        g.clamp(120,play.clampColor(20),20)
        n.move(30)
        g.up(10)
        g.clamp(182)
        n.move(-650)

sys.exit()
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
    n.goto(Vector(play.build_area[1]) + play.vectorColor((-200,280 + robot.dist_block)))
    #g.addJob(lambda: g.clamp(134,play.clampColor(-63)))
    g.clamp(130,play.clampColor(60))
    g.quiet()
    n.cap(play.capColor(-math.pi/2))
with MotorConfig(n, 300, 220):
    n.move_contact(0, 190)
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
n.move(-650)
n.motors.disable()
