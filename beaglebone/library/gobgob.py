"""
 gobgob clamp and dc motor api
"""
import struct
import array
import time
import asyncjob
from rampprofile import RampProfile
import threading

CMD_PWM_A = 0x1
CMD_PWM_B = 0x2
CMD_RST_A = 0x3
CMD_RST_B = 0x4
# pid
CMD_PID_A = 0x5
CMD_PID_B  = 0x6
CMD_GOTO_A = 0x7
CMD_GOTO_B = 0x8
CMD_ENABLE_A = 0x9
CMD_ENABLE_B = 0xA
CMD_MAX_PWM_A = 0xB
CMD_MAX_PWM_B = 0xC
CMD_INVERT_A = 0xD
CMD_INVERT_B = 0xE

#read requests
CMD_GET_POS_A =   0x21
CMD_GET_POS_B =   0x22
CMD_GET_PWM_A = 0x23
CMD_GET_PWM_B = 0x24

class ClampConfig:
    def __init__(self, gobgob, pwm=None , pid=None, speed=None
                     , pwm_clamp=None , pid_clamp=None, speed_clamp=None
                     , pwm_up=None , pid_up=None, speed_up=None ):
        
        self.gobgob = gobgob
        self.pwm_clamp = None
        self.pid_clamp = None
        self.speed_clamp = None
        self.pwm_up = None
        self.pid_up = None
        self.speed_up = None
        
        if pwm is not None:
            self.pwm_clamp = pwm
            self.pwm_up = pwm
        if pid is not None:
            self.pid_clamp = pid
            self.pid_up = pid
        if speed is not None:
            self.speed_clamp = speed
            self.speed_up = speed

        if pwm_clamp is not None:
            self.pwm_clamp = pwm_clamp
        if pid_clamp is not None:
            self.pid_clamp = pid_clamp
        if speed_clamp is not None:
            self.speed_clamp = speed_clamp

        if pwm_up is not None:
            self.pwm_up = pwm_up
        if pid_up is not None:
            self.pid_up = pid_up
        if speed_up is not None:
            self.speed_up = speed_up
        
    def __enter__(self):
        if self.pwm_clamp is not None:
            self.pwm_clamp_bkp = self.gobgob.motD.max_pwm
            self.gobgob.motD.max_pwm = self.pwm_clamp
            self.gobgob.motG.max_pwm = self.pwm_clamp
        if self.pid_clamp is not None:
            self.pid_clamp_bkp = self.gobgob.motD.pid.pid
            self.gobgob.motD.pid.pid = self.pid_clamp
            self.gobgob.motG.pid.pid = self.pid_clamp
        if self.speed_clamp is not None:
            self.speed_clamp_bkp = self.gobgob.speed_clamp
            self.gobgob.speed_clamp = self.speed_clamp

        if self.pwm_up is not None:
            self.pwm_up_bkp = self.gobgob.motH.max_pwm
            self.gobgob.motH.max_pwm = self.pwm_up
        if self.pid_up is not None:
            self.pid_up_bkp = self.gobgob.motH.pid.pid
            self.gobgob.motH.pid.pid = self.pid_up
        if self.speed_up is not None:
            self.speed_up_bkp = self.gobgob.speed_up
            self.gobgob.speed_up = self.speed_up
            
    def __exit__(self, type, value, traceback):
        if self.pwm_clamp is not None:
            self.gobgob.motD.max_pwm = self.pwm_clamp_bkp
            self.gobgob.motG.max_pwm = self.pwm_clamp_bkp
        if self.pid_clamp is not None:
            self.gobgob.motD.pid.pid = self.pid_clamp_bkp
            self.gobgob.motG.pid.pid = self.pid_clamp_bkp
        if self.speed_clamp is not None:
            self.gobgob.speed_clamp = self.speed_clamp_bkp

        if self.pwm_clamp is not None:
            self.gobgob.motH.max_pwm = self.pwm_up_bkp
        if self.pid_up is not None:
            self.gobgob.motH.pid.pid = self.pid_up_bkp
        if self.speed_up is not None:
            self.gobgob.speed_up = self.speed_up_bkp

class Pid(object):
    def __init__(self, parent, kp = 6.6, ki = 0.0, kd = 0.35, reverse = 0):
        self.parent = parent
        self.i2c = parent.i2c
        self.slave = parent.slave
        self.disable()
        #self.kp = kp
        #self.ki = ki 
        #self.kd = kd
        self.pid = (kp, ki, kd)
        self.reverse = reverse
        
    @property
    def pid(self):
        return self.kp, self.ki, self.kd
    
    @pid.setter
    def pid(self, coef):
        if coef[0] is not None:
            self.kp = coef[0]
        if coef[1] is not None:
            self.ki = coef[1]
        if coef[2] is not None:
            self.kd = coef[2]
    
        buff = struct.unpack("12B", struct.pack("fff", self.kp, self.ki, self.kd))
        self.i2c.write(self.slave, list((self.parent.CMD_PID,) + buff))
    
    @property
    def reverse(self):
        return self._reverse

    @reverse.setter
    def reverse(self, reverse):
        self._reverse = reverse
        self.i2c.write(self.slave, [self.parent.CMD_INVERT, reverse])
    
    def enable(self, enable = 1):
        self.i2c.write(self.slave, [self.parent.CMD_ENABLE, enable])

    def disable(self):
        self.enable(0)

class Motor(object):
    def __init__(self, i2c, slave,  motor = 'A', mm_to_tick = 200.0, reverse = 0):
        self.motor = motor
        self.i2c = i2c
        self.slave = slave
        self.mm_to_tick = mm_to_tick
        
        if self.motor == 'A':
            self.CMD_PWM = CMD_PWM_A
            self.CMD_RST = CMD_RST_A
            self.CMD_PID = CMD_PID_A
            self.CMD_GOTO = CMD_GOTO_A
            self.CMD_ENABLE = CMD_ENABLE_A
            self.CMD_MAX_PWM = CMD_MAX_PWM_A
            self.CMD_INVERT = CMD_INVERT_A
            
            self.CMD_GET_POS = CMD_GET_POS_A
            self.CMD_GET_PWM = CMD_GET_PWM_A
        else:
            self.CMD_PWM = CMD_PWM_B
            self.CMD_RST = CMD_RST_B
            self.CMD_PID = CMD_PID_B
            self.CMD_GOTO = CMD_GOTO_B
            self.CMD_ENABLE = CMD_ENABLE_B
            self.CMD_MAX_PWM = CMD_MAX_PWM_B
            self.CMD_INVERT = CMD_INVERT_B
            
            self.CMD_GET_POS = CMD_GET_POS_B
            self.CMD_GET_PWM = CMD_GET_PWM_B
        
        self.pid = Pid(self, reverse = reverse)
        self.target = 0
        self.max_pwm = 1.0

    @property
    def pwm(self):
        ret = self.i2c.readTransaction(self.slave,self.CMD_GET_PWM, 2)
        return struct.unpack('h', array.array('B', ret))[0]
    
    @pwm.setter 
    def pwm(self, pwm):
        buff = struct.unpack("2B", struct.pack("h", pwm))
        self.i2c.write(self.slave, list((self.CMD_PWM,) + buff))

    @property
    def max_pwm(self):
        return self._max_pwm
    
    @max_pwm.setter 
    def max_pwm(self, max_pwm):
        if max_pwm < 0.0:
            max_pwm = 0.0
        if max_pwm > 1.0:
            max_pwm = 1.0   
        self._max_pwm = max_pwm
        
        buff = struct.unpack("2B", struct.pack("h", self._max_pwm * 1024))
        self.i2c.write(self.slave, list((self.CMD_MAX_PWM,) + buff))
    
    @property
    def pos(self):
        ret = self.i2c.readTransaction(self.slave,self.CMD_GET_POS, 4)
        return struct.unpack('i', array.array('B', ret))[0] / self.mm_to_tick           
        
    @pos.setter
    def pos(self, pos):
        buff = struct.unpack("4B", struct.pack("i", pos * self.mm_to_tick))
        self.i2c.write(self.slave, list((self.CMD_RST,) + buff))
    
    @property
    def target(self):
        return self._target
    
    @target.setter
    def target(self, target):
        self._target = target
        buff = struct.unpack("4B", struct.pack("i", target * self.mm_to_tick))
        self.i2c.write(self.slave, list((self.CMD_GOTO,) + buff))
    
    def enable(self, enable = 1):
        if enable == 0:
            self.pid.disable()
            self.pwm = 0      
        else:
            self.pid.enable()
    
    def disable(self):
        self.enable(0)

class Gobgob(asyncjob.AsyncJob):
    def __init__(self, i2c):
        asyncjob.AsyncJob.__init__(self, "Gobgob")
        self.lock = threading.RLock()
        self.speed_up = 100
        self.speed_clamp = 135.0
        self.acc = 40.0
        self.motD = Motor(i2c, 14, 'A', mm_to_tick = 20.0, reverse = 0)
        self.motG = Motor(i2c, 12, 'B', mm_to_tick = 20.0, reverse = 1)
        self.motH = Motor(i2c, 12, 'A', mm_to_tick = -30.0)
        self.disable()
        pos_init = (30, -30, 0)
        self.pos = pos_init
        self.target = pos_init
        self.disable()
        
    @property
    def posH(self):
        with self.lock:
            return self.motH.pos           
        
    @posH.setter
    def posH(self, pos):
        with self.lock:
            delta = pos - self.motH.pos
            self.motH.pos = pos
            self.motD.pos -= delta
            self.motG.pos += delta

    @property
    def posD(self):
        with self.lock:
            return self.motD.pos + self.motH.pos           
        
    @posD.setter
    def posD(self, pos):
        with self.lock:
            self.motD.pos = pos - self.motH.pos

    @property
    def posG(self):
        with self.lock:
            return self.motG.pos - self.motH.pos           
        
    @posG.setter
    def posG(self, pos):
        with self.lock:
            self.motG.pos = pos + self.motH.pos  
    
    @property
    def pos(self):
        with self.lock:
            d = self.motD.pos
            g = self.motG.pos
            h = self.motH.pos
            return d + h, g - h, h            
        
    @pos.setter
    def pos(self, pos):
        with self.lock:
            self.posD = pos[0]
            self.posG = pos[1]
            self.posH = pos[2]
        
    @property
    def targetH(self):
        with self.lock:
            return self.motH.target           
        
    @targetH.setter
    def targetH(self, target):
        with self.lock:
            self.motH.target = target

    @property
    def targetD(self):
        with self.lock:
            return self.motD.target + self.motH.target           
        
    @targetD.setter
    def targetD(self, target):
        with self.lock:
            self.motD.target = target - self.motH.target
    
    @property
    def targetG(self):
        with self.lock:
            return self.motG.target - self.motH.target           
        
    @targetG.setter
    def targetG(self, target):
        with self.lock:
            self.motG.target = target + self.motH.target
    
    @property
    def target(self):
        with self.lock:
            return self.targetD, self.targetG, self.targetH           
        
    @target.setter
    def target(self, target):
        with self.lock:
            self.targetH = target[2]
            self.targetD = target[0]
            self.targetG = target[1]
    
    def enable(self, enable = 1):
        with self.lock:
            self.target = self.pos
            self.motD.enable(enable)
            self.motG.enable(enable)
            self.motH.enable(enable)
    
    def disable(self):
        with self.lock:
            self.enable(0)
    
    def quiet(self):
        with self.lock:
            self.target = self.pos
            time.sleep(0.1)
            self.target = self.pos
    
    def move(self, target, speed = None, acc = None):
        dt = 0.04
        if speed == None:
            speed_up = self.speed_up
            speed_clamp = self.speed_clamp
        else:
            speed_up = speed
            speed_clamp = speed
        if acc == None:
            acc = self.acc
        speed_d = speed_clamp * dt
        speed_g = speed_clamp * dt
        speed_h = speed_up * dt
        acc = acc * dt
        d,g,h = self.pos
        rampD = RampProfile(d, target[0], speed_d, acc)
        rampG = RampProfile(g, target[1], speed_g, acc)
        rampH = RampProfile(h, target[2], speed_h, acc)
        
        while self.abort == False:
            d = rampD.next()
            g = rampG.next()
            h = rampH.next()
            if not any([d, g, h]):
                break
            if d == None: d = target[0]
            if g == None: g = target[1]
            if h == None: h = target[2]

            self.target = (d, g, h)
            time.sleep(dt)
            
    def moveMonitor(self, target, speed = None, acc = None):
        with open("/tmp/toto", "w") as f:
            dt = 0.04
            if speed == None:
                speed_up = self.speed_up
                speed_clamp = self.speed_clamp
            else:
                speed_up = speed
                speed_clamp = speed
            if acc == None:
                acc = self.acc
            speed_d = speed_clamp * dt
            speed_g = speed_clamp * dt
            speed_h = speed_up * dt
            acc = acc * dt
            d,g,h = self.pos
            rampD = RampProfile(d, target[0], speed_d, acc)
            rampG = RampProfile(g, target[1], speed_g, acc)
            rampH = RampProfile(h, target[2], speed_h, acc)
            
            while True:
                d = rampD.next()
                g = rampG.next()
                h = rampH.next()
                if not any([d, g, h]):
                    break
                if d == None: d = target[0]
                if g == None: g = target[1]
                if h == None: h = target[2]
    
                self.target = (d, g, h)
                for i in range(5):
                    pd,pg,ph = self.pos
                    f.write("%f %f %f %f %f %f\n" % (d,g,h,pd,pg,ph))
                    time.sleep(dt/5)
        
    
    def up(self, haut, speed = None):
        d, g, h = self.pos
        h = haut
        self.move((d, g, h), speed)
    
    def clamp(self, e, X = None, speed = None):
        d, g, h = self.pos
        if X == None:
            X = (d + g) / 2.0
        d = X + e/2.0
        g = X - e/2.0
        self.move((d, g, h), speed)
    
    def detectBlockage(self, axes = "dgh", delta = 30):
        while self.working:
            time.sleep(0.3)
            if self.working == False:
                break
            dt, gt, ht = self.target
            d, g, h = self.pos
            print dt, gt, ht, "vs", d, g, h
            if 'd' in axes:
                if abs(d - dt) > delta:
                    return 'd'
            if 'g' in axes:
                if abs(g - gt) > delta:
                    return 'g'
            if 'h' in axes:
                if abs(h - ht) > delta:
                    return 'h'  
        return None
            
    
    def calibration(self):
        # calib H
        print "Calib H"
        self.quiet()
        self.motD.max_pwm = 0.8
        self.motG.max_pwm = 0.8
        self.motH.max_pwm = 0.8
        print "max_pwm", self.motD.max_pwm, self.motG.max_pwm, self.motH.max_pwm
        with ClampConfig(self, pwm_clamp = 0.38, pwm_up= 0.45 , speed = 70):
            print "max_pwm", self.motD.max_pwm, self.motG.max_pwm, self.motH.max_pwm
            self.addJob(lambda: self.up(self.posH - 160))
            
            if self.detectBlockage('h'):
                self.abort = True
                print "ABORT"

            self.waitJob()
            self.quiet()
            self.posH = 20 - 8 # 2cm - 8mm slider
            print "posH", self.posH
            print self.pos
        
#         with ClampConfig(self, pwm_clamp = 0.42, pwm_up= 0.8 , speed = 70):
#             self.addJob(lambda: do_print("start job"))
#             self.addJob(lambda: self.clamp(650))
#             self.addJob(lambda: do_print("end job"))
#             
#             if self.detectBlockage("dg"):
#                 self.abort = True
#                 print "ABORT"
# 
#             self.waitJob()
#             self.quiet()
        #calib D
        print "Calib D"
        time.sleep(0.5)
        with ClampConfig(self, pwm_clamp = 0.55, pwm_up= 0.69 , speed = 72):
            print "max_pwm", self.motD.max_pwm, self.motG.max_pwm, self.motH.max_pwm
            print "from", self.pos
            d, g, h = self.pos
            d += 400
            g += 15
            h = 20 - 8
            print "to", d, g, h
            self.addJob(lambda: self.move((d,g,h)))
            if self.detectBlockage("d"):
                self.abort = True
                print "ABORT"

            self.waitJob()
            self.quiet()
            self.posD = 305 / 2.0
            print "posD", self.posD

        #calib G
        print "Calib G"
        time.sleep(0.5)
        with ClampConfig(self, pwm_clamp = 0.55, pwm_up= 0.69 , speed = 72):
            print "from", self.pos
            d, g, h = self.pos
            g -= 400
            d -= 15
            h = 20 - 8
            print "to", d, g, h
            self.addJob(lambda: do_print("start job"))
            self.addJob(lambda: self.move((d,g,h)))
            self.addJob(lambda: do_print("end job"))
             
            if self.detectBlockage("g"):
                self.abort = True
                print "ABORT"
 
            self.waitJob()
            self.quiet()            
            self.posG = - 305 / 2.0
            print "posG", self.posG
            self.clamp(75,0)
            self.up(20)
            self.quiet()
         
            print "calib Done", self.pos
        self.disable()
        
def do_print(x):
    print x