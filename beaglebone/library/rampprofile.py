import math

class RampProfile(object):
    def __init__(self, start, end, max_speed, acc = 1, dec = None):
        self.pos = start
        self.target = end
        if self.target >= self.pos:
            self.sens = 1
        else:
            self.sens = -1
        self.max_speed = max_speed
        self.acc = acc
        if dec == None:
            dec = acc
        self.dec = dec
        
        self.speed = 0.0
        self.d_stop = 0.0
        
    def nextold(self):
        if self.pos == self.target:
            return None
        remaining = abs(self.target - self.pos)
        acc_speed = self.speed + self.acc
        dec_speed = math.sqrt(remaining * 2.0 * self.dec)
        
        self.speed = min(acc_speed, dec_speed, self.max_speed)
        
        if self.sens == 1:
            self.pos += self.speed
            if self.pos >= self.target:
                self.pos = self.target
                self.finished = True
        else:
            self.pos -= self.speed
            if self.pos <= self.target:
                self.pos = self.target

        return self.pos
    
    def next(self):
        if self.pos == self.target:
            return None
        remaining = abs(self.target - self.pos)
        self.d_stop = self.speed * self.speed / (2.0 * self.dec)
        if self.d_stop >= remaining:
            self.speed -= self.dec
        else: 
            self.speed += self.acc
        if self.speed > self.max_speed:
            self.speed = self.max_speed
        
        if self.sens == 1:
            self.pos += self.speed
            if self.pos > self.target:
                self.pos = self.target
                self.speed = 0.0
        else:
            self.pos -= self.speed
            if self.pos < self.target:
                self.pos = self.target
                self.speed = 0.0

        return self.pos

        
        
        