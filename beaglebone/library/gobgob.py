"""
 gobgob clamp and dc motor api
"""
import struct
import array
import time

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

class Pid(object):
    def __init__(self, parent, kp = 8.0, ki = 0.0, kd = 0.6, reverse = 0):
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
        self._target = 0
        self._max_pwm = 1.0
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

class Gobgob(object):
    def __init__(self, i2c):
        self.motD = Motor(i2c, 14, 'B', mm_to_tick = 20.0, reverse = 1)
        self.motG = Motor(i2c, 12, 'A', mm_to_tick = 20.0, reverse = 1)
        self.motH = Motor(i2c, 12, 'B', mm_to_tick = -30.0)
        self.disable()
        pos_init = (30, -30, 0)
        self.pos = pos_init
        self.target = pos_init
        self.disable()
        
    @property
    def posH(self):
        return self.motH.pos           
        
    @posH.setter
    def posH(self, pos):
        delta = pos - self.motH.pos
        self.motH.pos = pos
        self.motD.pos -= delta
        self.motG.pos += delta

    @property
    def posD(self):
        return self.motD.pos + self.motH.pos           
        
    @posD.setter
    def posD(self, pos):
        self.motD.pos = pos - self.motH.pos

    @property
    def posG(self):
        return self.motG.pos - self.motH.pos           
        
    @posG.setter
    def posG(self, pos):
        self.motG.pos = pos + self.motH.pos  
    
    @property
    def pos(self):
        return self.posD, self.posG, self.posH            
        
    @pos.setter
    def pos(self, pos):
        self.posD = pos[0]
        self.posG = pos[1]
        self.posH = pos[2]
        
    @property
    def targetH(self):
        return self.motH.target           
        
    @targetH.setter
    def targetH(self, target):
        self.motH.target = target

    @property
    def targetD(self):
        return self.motD.target + self.motH.target           
        
    @targetD.setter
    def targetD(self, target):
        self.motD.target = target - self.motH.target
    
    @property
    def targetG(self):
        return self.motG.target - self.motH.target           
        
    @targetG.setter
    def targetG(self, target):
        self.motG.target = target + self.motH.target
    
    @property
    def target(self):
        return self.targetD, self.targetG, self.targetH           
        
    @target.setter
    def target(self, target):
        self.targetH = target[2]
        self.targetD = target[0]
        self.targetG = target[1]
    
    def enable(self, enable = 1):
        self.motD.enable(enable)
        self.motG.enable(enable)
        self.motH.enable(enable)
    
    def disable(self):
        self.enable(0)
    
    def quiet(self):
        self.target = self.pos
    
    def up(self, target, speed = 2.0):
        d,g,h = self.pos
        if target > h:
            while h < target:
                h += speed
                self.target = (d,g,h)
                time.sleep(0.05)
        else:
            while h > target:
                h -= speed
                self.target = (d,g,h)
                time.sleep(0.05)
        self.target = (d,g,target)
        time.sleep(0.2)
        self.quiet()
    
        
        
