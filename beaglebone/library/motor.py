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

CMD_GET_STATUS =    0x21
CMD_GET_POS_A =     0x23
CMD_GET_POS_B =     0x24

SLAVE =     9

class Stepper:
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
            self.CMD_GET_STATUS_MASK = 1
            self.CMD_GET_POS = CMD_GET_POS_A
        else:
            self.CMD_MOVE_TO = CMD_MOVE_TO_B
            self.CMD_MOVE = CMD_MOVE_B
            self.CMD_STOP = CMD_STOP_B
            self.CMD_MAX_SPEED = CMD_MAX_SPEED_B
            self.CMD_ACC = CMD_ACC_B
            self.CMD_SET_POS = CMD_SET_POS_B
            self.CMD_ENABLE = CMD_ENABLE_B
            self.CMD_GET_STATUS_MASK = 2
            self.CMD_GET_POS = CMD_GET_POS_B         

    def move_to(self, pos):
        buff = struct.unpack("4B", struct.pack("I", pos))
        self.i2c.write(SLAVE, (self.CMD_MOVE_TO,) + buff)

    def move(self, pos):
        buff = struct.unpack("4B", struct.pack("I", pos))
        self.i2c.write(SLAVE, (self.CMD_MOVE,) + buff)

    def stop(self):
        self.i2c.write(SLAVE, [self.CMD_STOP])

    def get_status(self):
        ret = self.i2c.readTransaction(SLAVE,CMD_GET_STATUS, 1)
        return ret & self.CMD_GET_STATUS_MASK

    @property
    def pos(self):
        ret = self.i2c.readTransaction(SLAVE,self.CMD_GET_POS, 4)
        return struct.unpack('>I', array.array('B', ret))[0]           
        
    @pos.setter
    def pos(self, pos):
        buff = struct.unpack("4B", struct.pack("I", pos))
        self.i2c.write(SLAVE, (self.CMD_SET_POS,) + buff)

    @property
    def max_speed(self):
        """ read cached max_speed or i2c write it"""
        return self._max_speed

    @max_speed.setter
    def max_speed(self, speed):
        self._max_speed = speed
        buff = struct.unpack("4B", struct.pack("f", speed))
        self.i2c.write(SLAVE, (self.CMD_MAX_SPEED,) + buff)

    @property
    def acc(self):
        """ read cached acc or i2c write it"""
        return self._acc
    
    @acc.setter
    def acc(self, acc):
        self._acc = acc
        buff = struct.unpack("4B", struct.pack("f", acc))
        self.i2c.write(SLAVE, (self.CMD_ACC,) + buff)

    def enable(self):
        self.i2c.write(SLAVE, [self.CMD_ENABLE])