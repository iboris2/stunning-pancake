import rlcompleter, readline
readline.parse_and_bind('tab:complete')


import sys, time
sys.path.append('/root/bb/library')
from playground import *
from ihm import *
from navigation import *
import motor
import evitement
import bbio
import gobgob
import actuator
import robot
import threading
import math
from vector import *
import gametimer

class myi2c(object):
    def __init__(self, i2c, id=0):
        self.i2c = i2c
        self.id = id
        self.lock = threading.RLock()
        
    
    def write(self,a,b):
        with self.lock:
            nbtry = 4
            while nbtry:
                nbtry = nbtry - 1
                try:
                    ret = self.i2c.write(a, b)
                    return ret
                except:
                    print "i2c ERROR", self.id
                    pass
        
    
    def readTransaction(self,a,b,c):
        with self.lock:
            nbtry = 4
            while nbtry:
                nbtry = nbtry - 1
                try:
                    ret = self.i2c.readTransaction(a,b,c)
                    return ret
                except:
                    print "i2c ERROR", self.id
                    pass

_s = bbio.Wire1
_s2 = bbio.Wire2

_s.begin()
_s2.begin()

s = myi2c(_s, 1)
s2 = myi2c(_s2, 2)

def prepare_small():
    prepare_tower(0)
    
def prepare_big():
    prepare_tower(1)
       
def prepare_tower(big):
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
    with MotorConfig(n, 320, 220):
        dist = 102+30
        if big:
            dist += 35
        n.move(dist)
        g.clamp(185,0)

def fish(spot=0):
    print "fish"
    if strategy == 4 or strategy == 0:
        if spot == 0:
            spot = 1
        else:
            spot = 0
    n.goto(play.fish_spot[spot])
    n.cap(math.pi)
    with MotorConfig(n, 400, 620):
        n.move_contact(0,-play.fish_spot_contact)
    a.poisson.down()
    with MotorConfig(n, 670, 600):
        n.move(30)
        n.move(-26)
        n.turn(play.capColor(0.1),0,0)
        a.poisson.up()
        n.move(190)
        if strategy == 4 or strategy == 0:
            if spot == 1:
                n.goto(Vector(play.depose_spot[spot]) + play.vectorColor((0,-120)),30)
                n.move(-120)
            else:
                n.goto(play.depose_spot[spot],30)
        else:
            n.goto(play.depose_spot[spot],30)
        n.cap(math.pi, -35)
        x,y = n.position
        with MotorConfig(n, 300, 520):
            n.move_contact(0,x-2000)
            a.poisson.release()
        n.move(250)
        a.poisson.speedUp()#async

g = gobgob.Gobgob(s)
n = Navigation(motor.StepperBlock(s),evitement.Obstacle(s)) 
a = actuator.Actuator(s2)
a.umbrella.disable()
a.poisson.close()
a.poisson.disable()
n.motors.disable()
gt = gametimer.GameTimer(g, a, n, s, s2)
ihm = Ihm()


color, strategy = ihm.prepare()
play = Playground(color)
play.config(strategy)
n.position = play.start_pos
n.angle = play.start_angle
print "start pos", n.position, n.angle
with MotorConfig(n, 200, 150):
    n.move(22)
a.poisson.close()
n.motors.disable()
g.calibration()

##  #base config
n.blockage = Blockage.NONE
n.max_speed = 713
n.acc = 700
ihm.wait_starter()

gt.start()

try:
    g.addJob(lambda: g.clamp(290, 0,20))
    
    #g.up(80)
    #g.clamp(150, play.clampColor(-38),85+60)
    sand = Vector(play.sand[0])
    sand += play.vectorColor((0, robot.dist_clamp + 35))
    n.goto(sand)
    print "ma position"
    print n.position
    n.cap(play.capColor(-math.pi/2))
    
    with MotorConfig(n, 290, 240):
        g.addJob(lambda: time.sleep(0.2))
        g.addJob(lambda: g.clamp(120, 0,20))
        n.move(70)
    #prepare_small()
    dist_bord = 300
    with MotorConfig(n, 515, 420):
        n.goto(Vector(play.build_area[1]) + play.vectorColor((-210,dist_bord + robot.dist_block)),robot.rot_rayon)   
        n.cap(play.capColor(-math.pi/2),40)
        n.move(175)
        g.addJob(lambda: g.clamp(280))
        g.addJob(lambda: g.clamp(300,0,20))
        time.sleep(0.6)
    n.move(-300)
    
    offset = -104
    if strategy == 0: 
        n.goto(pos = play.seasheel[2], offset=offset)
        n.goto(pos = play.seasheel[1],rot_rayon=110, offset=offset, rotate_only=True)
        n.goto(pos = play.seasheel[1], offset=offset)
        n.goto(pos = play.seasheel[0],rot_rayon=80, offset=offset, rotate_only=True)
        n.goto(pos = play.seasheel[0], offset=offset)
    if strategy == 1:
        n.goto(pos = play.seasheel[3], offset=offset)
        n.goto(pos = play.seasheel[2],rot_rayon=30, offset=offset, rotate_only=True)  
        n.goto(pos = play.seasheel[2], offset=offset)
        n.goto(pos = play.seasheel[1],rot_rayon=110, offset=offset, rotate_only=True)
        n.goto(pos = play.seasheel[1], offset=offset)
        n.goto(pos = play.seasheel[0],rot_rayon=80, offset=offset, rotate_only=True)
        n.goto(pos = play.seasheel[0], offset=offset)
    if strategy == 2 or strategy == 3:
        n.goto(pos = play.seasheel[4], offset=offset)
        n.goto(pos = play.seasheel[3],rot_rayon=30, offset=offset, rotate_only=True)
        n.goto(pos = play.seasheel[3], offset=offset)
        n.goto(pos = play.seasheel[2],rot_rayon=110, offset=offset, rotate_only=True)  
        n.goto(pos = play.seasheel[2], offset=offset)
        n.goto(pos = play.seasheel_collect,rot_rayon=110, offset=offset, rotate_only=True)
    if strategy == 4:
        n.goto(pos = play.seasheel[2], offset=offset)
        n.goto(pos = play.seasheel[0],rot_rayon=80, offset=offset, rotate_only=True)
        n.goto(pos = play.seasheel[0], offset=offset)
        
    n.goto(pos = play.seasheel_collect, offset=offset)
    n.cap(play.capColor(math.pi/2),60)
    if strategy == 2 or strategy == 3:
        n.move_contact(0,45)
    
    n.move(-200)
    
        
    #close hut
    n.reculeto(play.close_hut)
    n.cap(0)
    with MotorConfig(n, 340, 620): 
        n.move_contact(0, -90)
        n.cap(0)
    n.move_contact(0, -50)
    n.move(350)
    
    
        
    #goto fish
    g.addJob(lambda: g.clamp(70,0,115))
    fish(0)
    fish(1)
    
    
    n.max_speed = 710
    n.acc = 735
    
    #goto next block
    n.goto(play.vectorColor((1450,910)))
    n.goto(play.vectorColor((650,910)))
    g.addJob(lambda: g.clamp(320,0,55))
    n.goto(Vector(play.sand[1])+Vector(450,0))
    n.cap(play.capColor(-math.pi))
    x,y = n.position
    print x, "cm du bord", y
    
    with MotorConfig(n, 590, 650): 
        n.move_contact(0, x - robot.dist_block + 50)
        g.clamp(120,play.clampColor(-70),55)
        g.up(52+48)
    x,y = n.position
    target = 375
    n.move(x - target)
    with MotorConfig(navigation=n, acc=950):
        n.turn(play.capColor(math.pi/2),play.capColor(robot.rot_rayon-50))
        n.move_contact(0, -50)
        n.reculeto(play.vectorColor((play.build_area[1][0]-200 - 150, 1050)), -20 )
        gt.openClampEnd(230)
    n.cap(play.capColor(-math.pi/2))
    x,y = n.position
    g.addJob(lambda: g.up(55))
    g.addJob(lambda: g.clamp(230))
    n.move(abs(y)-580)
    n.move(-40)
    print "END of programm"
except:
    pass 
print "END"
n.motors.disable()
g.disable()
while(1):
    time.sleep(1)
