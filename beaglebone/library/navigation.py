"""
 robot motion goto
"""
import math
import time
import odometry
from evitement import Obstacle
from vector import Vector

class Event(object):
    PRECISION = 0
    TIMEOUT = 1
    BLOCKAGE = 2
    OBSTACLE = 3
    def __init__(self, _type, value = 0):
        self.type = _type
        self.value = value
        if _type == 0:
            str = "PRECISION"
        elif _type == 1:
            str = "TIMEOUT"
        elif _type == 2:
            str = "BLOCKAGE"
        elif _type == 3:
            str = "OBSTACLE"
        print  "event", str, value

class Blockage(object):
    NONE = 0
    RIGHT = 1
    LEFT = 2
    BOTH = 3
    ANY = 4
    def __init__(self, odo):
        self.odo = odo
        self.reset(9999999,9999999)
        self.threshold = 20 # check every 20mm
        self.max_ratio = 0.5 # 10mm error / 20 mm
    
    def reset(self, rD, rG):
        self.rD = rD
        self.rG = rG
        self.odoD, self.odoG = self.odo.encoder  
    
    def detect(self, rD, rG):
        ret = Blockage.NONE
        odoD, odoG = self.odo.encoder
        delta_rD = self.rD - rD
        delta_rG = self.rG - rG
        #print "rdrg",rD,rG,"odo",odoD,odoG
        
        #print "delta:", delta_rD, "-",odoD - self.odoD,  delta_rG, "-",odoG - self.odoG
        if delta_rD >= self.threshold:
            delta_odoD = abs(odoD - self.odoD)
            #print "delta D", delta_rD, delta_odoD
            if delta_odoD / delta_rD < self.max_ratio:
                ret = ret | Blockage.RIGHT
            self.rD = rD
            self.odoD = odoD
        
        if delta_rG >= self.threshold:
            delta_odoG = abs(odoG - self.odoG)
            #print "delta G", delta_rG, delta_odoG
            if delta_odoG / delta_rG < self.max_ratio:
                ret = ret | Blockage.LEFT
            self.rG = rG
            self.odoG = odoG
        
        return ret
        
class MotorConfig:
    def __init__(self, navigation, acc=None, vmax=None):
        self.navigation = navigation
        self.acc = acc
        self.vmax = vmax
        
        self.acc_bkp = navigation.acc
        self.vmax_bkp = navigation.max_speed
        
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

class ObstacleConfig:
    def __init__(self, navigation, precision=None, blockage=None, obs_detection=None):
        self.navigation = navigation
        self.precision = precision
        self.blockage = blockage
        self.obs_detection = obs_detection
        
        self.precision_bkp = navigation.precision
        self.blockage_bkp = navigation.blockage
        self.obs_detection_bkp = navigation.obs_detection

    def __enter__(self):
        if self.precision is not None:
            self.navigation.precision = self.precision
        if self.blockage is not None:
            self.navigation.blockage = self.blockage
        if self.obs_detection is not None:
                self.navigation.obs_detection = self.obs_detection

    def __exit__(self, type, value, traceback):
        if self.precision is not None:
            self.navigation.precision = self.precision_bkp
        if self.blockage is not None:
            self.navigation.blockage = self.blockage_bkp
        if self.obs_detection is not None:
            self.navigation.obs_detection = self.obs_detection_bkp

class Navigation(object):
    def __init__(self, motors, _evitement):
        #136mm * PI = 1600 step
        self.mm_to_step = 80.0/79.0 * (1600 / (math.pi * 136)) / 2.0
        self.step_to_mm = 1.0/self.mm_to_step
        self.motors = motors
        
        self.D = 306.2
        self.rayon = self.D/2.0
        
        self.encoder_tick_to_mm = 0.07088165616
        self.rayon_encoder = 136.6092
        
        self.odo = odometry.Odometry(self.encoder_tick_to_mm, self.rayon_encoder)
        self.odo.position = (0,0)
        self.odo.angle = math.pi/2
        
        self.approach_speed = 90
        
        self.evitement = _evitement
        
        self.blockage_detection = Blockage(self.odo)
        
        #wait event params
        self.precision = 1
        self.blockage = Blockage.ANY
        self.obs_detection = Obstacle.NONE
        
    
    @property
    def max_speed(self):
        s = self.motors.max_speed
        return tuple([self.step_to_mm * x for x in s])

    @max_speed.setter
    def max_speed(self, speed):
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
        nbtry = 2
        while True:
            start = Vector(self.position)
            ev = self._move(mm)
            if ev.type is Event.PRECISION:
                return ev
            end = Vector(self.position)
            if (mm > 0.0):
                self.simpleEventReaction(ev, EventReaction.MOVING_FORWARD)
            else:
                self.simpleEventReaction(ev, EventReaction.MOVING_BACKWARD)
            after = Vector(self.position)
            if mm > 0.0:
                mm = mm - (end-start).norm() + (after-end).norm()
            else:
                mm = mm + (end-start).norm() - (after-end).norm()
            if nbtry == 0:
                return ev
            nbtry -= 1
            

    def _move(self, mm):
        if mm == 0.0:
            return Event(Event.PRECISION)
        obs_detection = self.obs_detection
        if mm > 0.0:
            obs_detection = obs_detection & Obstacle.FRONT
        else:
            obs_detection = obs_detection & Obstacle.BACK
        dist = -1 * mm * self.mm_to_step
        self.motors.move(dist, dist)
        print "move", mm
        return self.waitForEvent(obs_detection=obs_detection)

    def update_move(self, mm):
        if mm == 0.0:
            return Event(Event.PRECISION)
        dist = -1 * mm * self.mm_to_step
        self.motors.update_move(dist, dist)
        print "update_move", mm
        return self.waitForEvent()
    
    def move_contact(self, distA, distB, approach_speed=None, speed=None):
        if speed == None:
            speed = self.max_speed
            speed = (speed[0] + speed[1]) / 2.0
        acc = (self.acc[0] + self.acc[1]) / 2.0 
        if approach_speed == None:
            approach_speed = self.approach_speed
        
        precision = (speed*speed)/ (2.0 * acc)
        if distA != 0.0:
            with MotorConfig(self,vmax=speed):
                with ObstacleConfig(self, precision=precision, blockage=Blockage.ANY, obs_detection=Obstacle.NONE):
                    ret = self._move(distA)
                    if ret.type == Event.BLOCKAGE:
                        return ret
        with MotorConfig(self,vmax=speed):
            with ObstacleConfig(self, precision=1, blockage=Blockage.ANY, obs_detection=Obstacle.NONE):
                ret = self._move(distB)
                self.stop(True)
                return ret
    def cap(self, new_angle, rayon=0):
        print "cap", new_angle
        angle = self.angle
        new_angle = ClosestEquivalentAngle(angle, new_angle)
        diff_angle = new_angle - angle
        if rayon:
            if diff_angle > 0:
                rayon = -rayon
        return self.turn(diff_angle, rayon)
            

    def turn(self, angle, rayon=0, nbtry=2):
        new_angle = self.angle + angle
        while True:
            if angle == 0.0:
                return Event(Event.PRECISION)
            mA = angle * (rayon + self.rayon)
            mB = angle * (rayon - self.rayon)
            # compute speed and acc
            if abs(rayon) < 50:
                boost = 1.22
            else:
                boost = 1.0
            vmaxA, vmaxB = self.max_speed
            vmaxA = max(vmaxA,vmaxB)
            vmaxB = vmaxA
            accA, accB = self.acc
            accA = max(accA,accB) * boost
            accB = accA

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
                print "turn", angle, rayon 
                ret = self.waitForEvent(precision=0.5, obs_detection=0)
                error_angle = new_angle - self.angle 
                print "error cap",  error_angle
                if abs(error_angle) < 0.02:
                    return ret
                if nbtry <= 0:
                    return Event(Event.BLOCKAGE, Blockage.RIGHT | Blockage.LEFT) 
                nbtry = nbtry -1
                angle = error_angle
    
    def stop(self, emergency=False, backward=False):
        print "stop", emergency
        if emergency == True:
            if backward:
                conf = (1000,1000)
            else:
                conf = (3500,3500)
            with MotorConfig(self, conf):
                self.motors.stop()
                self.waitForEvent(precision=2, blockage=Blockage.NONE, obs_detection=Obstacle.NONE)
        else:
            self.motors.stop()
            self.waitForEvent(precision=2)
            
    def waitForEvent(self, timeout=15, precision=None, blockage=None, obs_detection=None):
        start = time.time()
        time.sleep(0.10)
        if precision == None:
            precision = self.precision
        if blockage == None:
            blockage = self.blockage
        rD, rG = self.motors.remaining() # check if not sending previou move remqining
        rD = abs(rD * self.step_to_mm)
        rG = abs(rG * self.step_to_mm)
        self.blockage_detection.reset(rD, rG)
        
        if obs_detection == None:
            obs_detection = self.obs_detection
        
        print "waitForEvent", timeout, precision, blockage, obs_detection

        while True :
            rD, rG = self.motors.remaining() # check if not sending previou move remqining
            rD = abs(rD * self.step_to_mm)
            rG = abs(rG * self.step_to_mm)

            if rD <= precision and rG <= precision:
                return Event(Event.PRECISION)
            
            if blockage:
                ret = self.blockage_detection.detect(rD, rG)
                if blockage <= Blockage.BOTH:
                    if ret & blockage == blockage: #if RIGHT or LEFT or RIGHTandLEFT
                        return Event(Event.BLOCKAGE, ret)
                elif ret:
                    return Event(Event.BLOCKAGE, ret)
            
            #self.checkEndOfGame()
            if obs_detection:
                ret = self.evitement.obstacleDetected(obs_detection)
                if ret is not Obstacle.NONE:
                    return Event(Event.OBSTACLE, ret)

            if(timeout > 0 and start + timeout < time.time() ):
                print ("!!! Timeout")
                return Event(Event.TIMEOUT)
            
            #todo blockage
            time.sleep(0.04);
    
    @property
    def position(self):
        return self.odo.position
        
    @position.setter
    def position(self, pos):
        self.odo.position = pos
        
    @property
    def angle(self):
        return self.odo.angle
        
    @angle.setter
    def angle(self, angle):
        self.odo.angle = angle
    
    def simpleEventReaction(self, event, prev_action):
        print EventReaction.to_string[prev_action]
        if event.type == Event.PRECISION:
            return
        if prev_action == EventReaction.MOVING_BACKWARD:
            self.stop(True,True)
        else:
            self.stop(True)
        dist = 50
        if prev_action == EventReaction.MOVING_FORWARD:
            dist = -dist
        with ObstacleConfig(self, 1, Blockage.NONE, Obstacle.NONE):
            self._move(dist)
    
    def eventReaction(self, event, prev_action):
        print EventReaction.to_string[prev_action]
        if event.type == Event.PRECISION:
            return
        if prev_action == EventReaction.MOVING_BACKWARD:
            self.stop(True,True)
        else:
            self.stop(True)
        angle = math.pi / 8
        dist = 50
        rayon = self.rayon
        if event.type == Event.BLOCKAGE:
            if prev_action == EventReaction.TURN_RIGHT or prev_action == EventReaction.TURN_LEFT:
                if event.value & Blockage.RIGHT:
                    if prev_action == EventReaction.TURN_LEFT:
                        angle = -angle
                        dist = -dist
                        rayon = -rayon
                        #self.turn(-angle, self.rayon)
                    else:
                        rayon = -rayon
                        #angle = angle
                        #dist = dist
                        #self.turn(angle, self. rayon)
                else: #event.value == Blockage.LEFT:
                    rayon = rayon
                    if prev_action == EventReaction.TURN_RIGHT:
                        #angle = angle
                        dist = -dist
                    else:
                        angle = -angle
                        #dist = dist
            if prev_action == EventReaction.MOVING_FORWARD:
                dist = - dist
                if event.value & Blockage.RIGHT:
                    angle = -angle
                    rayon = -rayon
                #else: event.value == Blockage.LEFT:
            if prev_action == EventReaction.MOVING_BACKWARD:
                if event.value & Blockage.RIGHT:
                    rayon = -rayon
                else:
                    angle = -angle
            with MotorConfig(self, (300,300), (300,300)):
                with ObstacleConfig(self, 1, Blockage.NONE, Obstacle.NONE):
                    self.turn(angle, rayon)
                    self.move(dist)

        if event.type == Event.OBSTACLE:         
            if event.value & Obstacle.RIGHT:
                angle = -angle
                dist = -dist
            elif event.value & Obstacle.LEFT:
                dist = -dist
            if event.value & Obstacle.BACK:
                angle = 0
            with MotorConfig(self, (300,300), (300,300)):
                with ObstacleConfig(self, 1, Blockage.NONE, Obstacle.NONE):
                    self.turn(angle, rayon)
                    self.move(dist)

    def reculeto(self, pos, rot_rayon=0, rotate_only = False, offset=0):
        return self.goto(pos,rot_rayon, True, rotate_only, offset)
        
    def goto(self, pos, rot_rayon=0, recule = False, rotate_only = False, offset=0):
        nbtry = 3
        while True:
            ev = self._goto(pos, rot_rayon, recule, rotate_only, offset)
            if ev.type is Event.PRECISION:
                return ev
            nbtry = nbtry - 1
            if nbtry == 0:
                return ev
        
                   
    
    def _goto(self, pos, rot_rayon, recule, rotate_only, offset):
        print "goto", pos
        x , y = self.position
        angle = self.angle
        if recule:
            angle += math.pi
        new_angle = math.atan2(pos[1] - y, pos[0] - x)
        new_angle = ClosestEquivalentAngle(angle, new_angle)
        diff_angle = new_angle - angle
        if rot_rayon:
            if diff_angle > 0:
                rot_rayon = -rot_rayon
            if recule:
                rot_rayon = -rot_rayon
        ev = self.turn(diff_angle, rot_rayon)
        if (diff_angle > 0.0):
            self.simpleEventReaction(ev, EventReaction.TURN_LEFT)
        else:
            self.simpleEventReaction(ev, EventReaction.TURN_RIGHT)    
        if ev.type is not Event.PRECISION:
            return ev
        if rotate_only:
            return ev
        x , y = self.position
        dist = math.sqrt(   (pos[1] - y)**2 + (pos[0] - x)**2)
        dist += offset
        if dist < 0.0:
            dist = 0.0
        if recule:
            dist = -dist
        ev = self._move(dist)
        if (dist > 0.0):
            self.simpleEventReaction(ev, EventReaction.MOVING_FORWARD)
        else:
            self.simpleEventReaction(ev, EventReaction.MOVING_BACKWARD)  
        return ev
class EventReaction():
    NO = 0
    TURN_RIGHT = 1
    TURN_LEFT = 2
    MOVING_FORWARD = 3
    MOVING_BACKWARD = 4
    to_string = ["?", "TURN_RIGHT", "TURN_LEFT", "MOVING_FORWARD", "MOVING_BACKWARD"]
    
    def __init__(self):
        pass

def ClosestEquivalentAngle(old_angle, new_angle):
    if new_angle<=old_angle :
        while (not (old_angle-math.pi<=new_angle and new_angle<old_angle+math.pi)):
            new_angle+=2*math.pi
        return new_angle
    else:
        while (not (old_angle-math.pi<=new_angle and new_angle<old_angle+math.pi)):
            new_angle-=2*math.pi
        return new_angle;
        