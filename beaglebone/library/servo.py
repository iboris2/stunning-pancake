import math, time
from rampprofile import RampProfile

PCA9685_MODE1 = 0x0
PCA9685_PRESCALE = 0xFE

PCA9685_SLEEP = 0x10
PCA9685_RESTART = 0x80

SERVO_L = 0x8
SERVO_H = 0x9

SERVO_MULTIPLIER = 4

class ServoError(Exception):
    pass


class PwmOutOfBoundsError(ServoError):
    pass


class ServoController(object):

    def __init__(self, i2c, freq = 60, pw_min=300, pw_max=600):
        self.i2c = i2c
        self.slave = 0x40
        prescale = ServoController.calc_prescale(freq)

        self.init_i2c(prescale)

        self.servos = []

        for idx in range(16):
            servo = Servo(self.i2c, idx, pw_min, pw_max)
            self.servos.append(servo)

    def init_i2c(self, prescale):
        try:
            old_mode = self.i2c.readTransaction(self.slave, PCA9685_MODE1, 1)[0]
            self.i2c.write(self.slave, [PCA9685_MODE1, (old_mode & 0x7F) | PCA9685_SLEEP]) # set sleep mode.
            self.i2c.write(self.slave, [PCA9685_PRESCALE, int(prescale)])
            self.i2c.write(self.slave, [PCA9685_MODE1, old_mode])
            time.sleep(0.01)
            self.i2c.write(self.slave, [PCA9685_MODE1, (old_mode | PCA9685_RESTART) &  0b11101111])
        except:
            print "servo init failed"

    @classmethod
    def calc_prescale(cls, freq):
        prescale = 25000000.0
        prescale /= 4096
        prescale /= freq
        prescale -= 1
        prescale += 0.5
        return math.floor(prescale)

class Servo(object):

    def __init__(self, i2c, servo_num, pw_min=300, pw_max=600):
        self.i2c = i2c
        self.slave = 0x40
        self.servo_num = servo_num

        self.configure( )

        self.low_addr = SERVO_L + (servo_num * SERVO_MULTIPLIER)
        self.high_addr = SERVO_H + (servo_num * SERVO_MULTIPLIER)

    def set_pwm(self, pwm):
        print pwm
        if pwm < self.pw_min:
            pwm = self.pw_min
        if pwm > self.pw_max:
            pwm = self.pw_max

        self.i2c.write(self.slave, [self.low_addr, pwm & 0xff])
        self.i2c.write(self.slave, [self.high_addr, pwm >> 8])
    
    def configure(self, speed=85.0, acc=12.0, angle_min = -90, angle_max = 90, pw_min=300, pw_max=600):
        self.speed = speed
        self.acc = acc
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.pw_min = pw_min
        self.pw_max = pw_max
    
    @property
    def angle(self):
        low = self.i2c.readTransaction(self.slave, self.low_addr, 1)[0]
        high = self.i2c.readTransaction(self.slave, self.high_addr, 1)[0]
        pw = low + (high * 256)
        val = float(pw - self.pw_min) / float(self.pw_max - self.pw_min)
        return val * (self.angle_max - self.angle_min) + self.angle_min
    
    @angle.setter
    def angle(self, angle):
        val = float(angle - self.angle_min) / float(self.angle_max - self.angle_min)
        self.set_pwm(int(val * (self.pw_max - self.pw_min) + self.pw_min))
    
    def move(self, angle, speed = None, acc = None):
        dt = 0.05
        if speed == None:
            speed = self.speed
        if acc == None:
            acc = self.acc
        speed = speed * dt
        acc = acc * dt
        ramp = RampProfile(self.angle, angle, speed, acc)
        while True:
            pos = ramp.next()
            if not pos:
                break
            self.angle = pos
            print pos
            time.sleep(dt)

    def disable(self):
        self.i2c.write(self.slave, [self.low_addr, 0])
        self.i2c.write(self.slave, [self.high_addr, 0])

