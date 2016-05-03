import struct, array

class IrSensor(object):
    CMD_GET_IR_A = 0x25
    CMD_GET_IR_B = 0x26

    def __init__(self, i2c, slave, name = 'A', threshold = 200):
        self.i2c = i2c
        self.slave = slave
        self.name = name
        self.threshold = threshold
        if self.name == 'A':
            self.CMD_GET_IR = IrSensor.CMD_GET_IR_A
        else:
            self.CMD_GET_IR = IrSensor.CMD_GET_IR_B  

    @property
    def value(self):
        ret = self.i2c.readTransaction(self.slave,self.CMD_GET_IR, 2)
        return struct.unpack('h', array.array('B', ret))[0]
    
    def obstacleDetected(self):
        if self.value > self.threshold:
            return True
        return False

class Obstacle(object):
    NONE = 0
    RIGHT = 1
    LEFT = 2
    FRONT = 3
    BACK = 4
    BOTH = FRONT | BACK
    def __init__(self, i2c):
        self.ir_right = IrSensor(i2c, 12, 'B')
        self.ir_left = IrSensor(i2c, 12, 'A')
        self.ir_back = IrSensor(i2c, 14, 'A')
    
    def obstacleDetected(self, direction = 3):
        ret = Obstacle.NONE
        if direction & Obstacle.RIGHT:
            if self.ir_right.obstacleDetected():
                ret = ret | Obstacle.RIGHT
        if direction & Obstacle.RIGHT:
            if self.ir_left.obstacleDetected():
                ret = ret | Obstacle.LEFT
        if direction & Obstacle.BACK:
            if self.ir_back.obstacleDetected():
                ret = ret | Obstacle.BACK
        return ret
