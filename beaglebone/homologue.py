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
s = bbio.Wire1
s2 = bbio.Wire2

s.begin()
s2.begin()

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
n.move(10)
n.motors.disable()
ihm.wait_starter()
#take sand
sand = Vector(play.sand[0])
sand += play.vectorColor((0, robot.dist_front + 30))
n.goto(sand)
n.cap(play.capColor(-math.pi/2))
time.sleep(2)
with MotorConfig(n, 150, 150):
    n.move(100)
sys.exit()
with MotorConfig(n, 400, 150):
    n.goto(Vector(play.build_area[1]) + play.vectorColor(-200,160))
    n.cap(-math.pi/2)
with MotorConfig(n, 200, 150):
    n.move_contact( distB = 160)
    
