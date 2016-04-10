""" 
interface with i2c stepper driver board
"""

import struct
import array

""" 
i2c request
"""
CMD_MOVE_TO_A =     0x1
CMD_MOVE_TO_B =     0x2
CMD_MOVE_A =        0x3
CMD_MOVE_B =        0x4
CMD_STOP_A =        0x5
CMD_STOP_B =        0x6
CMD_MAX_SPEED_A =   0x7
CMD_MAX_SPEED_B =   0x8
CMD_ACC_A =         0x9
CMD_ACC_B =         0xA
CMD_SET_POS_A =     0xB
CMD_SET_POS_B =     0xC
CMD_ENABLE_A =      0xD
CMD_ENABLE_B =      0xE

CMD_REMAINING_A =   0x21
CMD_REMAINING_B =   0x22
CMD_GET_POS_A =     0x23
CMD_GET_POS_B =     0x24

SLAVE =     9

class StepperBlock(object):
    """ 
    2 steppers
    """
    def __init__(self, i2c):
        self.motorA = Stepper(i2c, 'A')
        self.motorB = Stepper(i2c, 'B')
        self.i2c = i2c

        self.CMD_MOVE_TO = CMD_MOVE_TO_A
        self.CMD_MOVE = CMD_MOVE_A
        self.CMD_STOP = CMD_STOP_A
        self.CMD_MAX_SPEED = CMD_MAX_SPEED_A
        self.CMD_ACC = CMD_ACC_A
        self.CMD_SET_POS = CMD_SET_POS_A
        self.CMD_ENABLE = CMD_ENABLE_A
        self.CMD_GET_POS = CMD_GET_POS_A
        self.CMD_REMAINING = CMD_REMAINING_A

    def move_to(self, posA, posB):
        buff = struct.unpack("8B", struct.pack("ii", posA, posB))
        self.i2c.write(SLAVE, list((self.CMD_MOVE_TO,) + buff))

    def move(self, posA, posB):
        buff = struct.unpack("8B", struct.pack("ii", posA, posB))
        self.i2c.write(SLAVE, list((self.CMD_MOVE,) + buff))

    def stop(self):
        self.i2c.write(SLAVE, [self.CMD_STOP, 0])

    def remaining(self):
        ret = self.i2c.readTransaction(SLAVE,self.CMD_REMAINING, 8)
        return struct.unpack('ii', array.array('B', ret))

    @property
    def pos(self):
        ret = self.i2c.readTransaction(SLAVE,self.CMD_GET_POS, 8)
        return struct.unpack('ii', array.array('B', ret))           
        
    @pos.setter
    def pos(self, pos):
        buff = struct.unpack("8B", struct.pack("ii", pos[0], pos[1]))
        self.i2c.write(SLAVE, list((self.CMD_SET_POS,) + buff))

    @property
    def max_speed(self):
        """ read cached max_speed or i2c write it"""
        return self.motorA._max_speed, self.motorB._max_speed

    @max_speed.setter
    def max_speed(self, speed):
        self.motorA._max_speed = speed[0]
        self.motorB._max_speed = speed[1]
        buff = struct.unpack("8B", struct.pack("ff", speed[0], speed[1]))
        self.i2c.write(SLAVE, list((self.CMD_MAX_SPEED,) + buff))

    @property
    def acc(self):
        """ read cached acc or i2c write it"""
        return self.motorA._acc, self.motorB._acc
    
    @acc.setter
    def acc(self, acc):
        self.motorA._acc = acc[0]
        self.motorB._acc = acc[1]
        buff = struct.unpack("8B", struct.pack("ff", acc[0], acc[1]))
        self.i2c.write(SLAVE, list((self.CMD_ACC,) + buff))

    def enable(self, enable = (1, 1)):
        self.i2c.write(SLAVE, list((self.CMD_ENABLE,) + enable))

    def disable(self, enable = (0, 0)):
        self.enable(enable);

class Stepper(object):
    def __init__(self, i2c,  motor = 'A'):
        self.motor = motor
        self.i2c = i2c
        
        self._acc = 1600.0
        self._max_speed = 400* 2.5
        
        if self.motor == 'A':
            self.CMD_MOVE_TO = CMD_MOVE_TO_A
            self.CMD_MOVE = CMD_MOVE_A
            self.CMD_STOP = CMD_STOP_A
            self.CMD_MAX_SPEED = CMD_MAX_SPEED_A
            self.CMD_ACC = CMD_ACC_A
            self.CMD_SET_POS = CMD_SET_POS_A
            self.CMD_ENABLE = CMD_ENABLE_A
            self.CMD_REMAINING = CMD_REMAINING_A
            self.CMD_GET_POS = CMD_GET_POS_A
        else:
            self.CMD_MOVE_TO = CMD_MOVE_TO_B
            self.CMD_MOVE = CMD_MOVE_B
            self.CMD_STOP = CMD_STOP_B
            self.CMD_MAX_SPEED = CMD_MAX_SPEED_B
            self.CMD_ACC = CMD_ACC_B
            self.CMD_SET_POS = CMD_SET_POS_B
            self.CMD_ENABLE = CMD_ENABLE_B
            self.CMD_REMAINING = CMD_REMAINING_B
            self.CMD_GET_POS = CMD_GET_POS_B        

    def move_to(self, pos):
        buff = struct.unpack("4B", struct.pack("i", pos))
        self.i2c.write(SLAVE, list((self.CMD_MOVE_TO,) + buff))

    def move(self, pos):
        buff = struct.unpack("4B", struct.pack("i", pos))
        self.i2c.write(SLAVE, list((self.CMD_MOVE,) + buff))

    def stop(self):
        self.i2c.write(SLAVE, [self.CMD_STOP])

    def remaining(self):
        ret = self.i2c.readTransaction(SLAVE,self.CMD_REMAINING, 4)
        return struct.unpack('I', array.array('B', ret))[0]

    @property
    def pos(self):
        ret = self.i2c.readTransaction(SLAVE,self.CMD_GET_POS, 4)
        return struct.unpack('I', array.array('B', ret))[0]           
        
    @pos.setter
    def pos(self, pos):
        buff = struct.unpack("4B", struct.pack("i", pos))
        self.i2c.write(SLAVE, list((self.CMD_SET_POS,) + buff))

    @property
    def max_speed(self):
        """ read cached max_speed or i2c write it"""
        return self._max_speed

    @max_speed.setter
    def max_speed(self, speed):
        self._max_speed = speed
        buff = struct.unpack("4B", struct.pack("f", speed))
        self.i2c.write(SLAVE, list((self.CMD_MAX_SPEED,) + buff))

    @property
    def acc(self):
        """ read cached acc or i2c write it"""
        return self._acc
    
    @acc.setter
    def acc(self, acc):
        self._acc = acc
        buff = struct.unpack("4B", struct.pack("f", acc))
        self.i2c.write(SLAVE, list((self.CMD_ACC,) + buff))

    def enable(self):
        self.i2c.write(SLAVE, [self.CMD_ENABLE, 1])

    def disable(self):
        self.i2c.write(SLAVE, [self.CMD_ENABLE, 0])