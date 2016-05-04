import math, vector
import robot
class Playground(object):
    def __init__(self, color='purple'):
        self.color = color
        
        if self.color == 'purple':
            invert = - 1
        else:
            invert = 1
        self.invert = invert
        #robot 7 x 20.25
        #28 x 40.5
        self.start_pos = [1100-203, (1500 - 70) * invert]
        self.start_angle = - math.pi/2 * invert
        
        self.all_seashell = []
        self.all_seashell.append([1250, 1250 * invert]) # 0
        self.all_seashell.append([1500, 1250 * invert]) #1
       
        self.all_seashell.append([1250, 800 * invert]) #2
        self.all_seashell.append([1550, 800 * invert]) #3
        self.all_seashell.append([1850, 800 * invert]) #4
       
        self.all_seashell.append([1450, 600 * invert]) #5
        self.all_seashell.append([1650, 300 * invert]) #6
       
        self.all_seashell.append([1550, 0 * invert]) #7
        self.all_seashell.append([1850, 0 * invert]) #8
       
        self.all_seashell.append([1650, -300 * invert]) #9
        
        self.seasheel_collect = [850, 1180 * invert]
        
        # door
        self.hut = [
                    [0, 1200 * invert],
                    [0, 900 * invert] 
                    ]
        self.close_hut = (110, (1200+900)* invert/2.0)
        #fish spot
        self._fish_area = [ 
                          [2000, 1000 * invert],                    
                          [2000, 600 * invert]
                          ]
        e = 70
        ecart_fish_spot = 300
        self.fish_spot_contact = ecart_fish_spot - robot.dist_back + 30
        self.fish_spot = [
                          [2000-ecart_fish_spot, (820 + e) * invert],                    
                          [2000-ecart_fish_spot, (820 - e) * invert]
                          ]
        self._depose_area = [ 
                          [2000-ecart_fish_spot, 550 * invert],                    
                          [2000-ecart_fish_spot, 450 * invert]
                          ]
        ecart_bord = 180
        ecart_spot = 150
        self.depose_spot = [
                          [2000-250, (550 - ecart_bord) * invert],                    
                          [2000-250, (550 - ecart_bord - ecart_spot) * invert]
                          ]
        
        #sand
        self.sand = [
                     [900, 850 * invert],
                     [58, 666 * invert],
                     [58 * 2, 0]
                     ]
        
        self.build_area = [[750 + 22, 24 * invert],
                      [750 + 22+ 600, 24 * invert]
                      ]
        

    def config(self, conf):
        if conf == 0:
            self.seasheel = [
                            self.all_seashell[0], self.all_seashell[1],
                            self.all_seashell[5],
                            self.all_seashell[7],self.all_seashell[8],
                            self.all_seashell[9]
                            ]
        elif conf == 1:
            self.seasheel = [
                            self.all_seashell[0], self.all_seashell[1],
                            self.all_seashell[5], self.all_seashell[6],
                            self.all_seashell[7],self.all_seashell[8]
                            ]

        elif conf == 2:
            self.seasheel = [
                            self.all_seashell[0], self.all_seashell[1],
                            self.all_seashell[2], self.all_seashell[3],
                            self.all_seashell[6]
                            ]
        elif conf == 3:
            self.seasheel = [
                            self.all_seashell[0], self.all_seashell[1],
                            self.all_seashell[2], self.all_seashell[3],
                            self.all_seashell[6]
                            ]
           
        else: # if conf == 4:
            self.seasheel = [
                            self.all_seashell[0], self.all_seashell[1],
                            self.all_seashell[2], self.all_seashell[3], self.all_seashell[4]
                            ]
        
    def vectorColor(self,val):
        return vector.Vector(val[0], val[1] * self.invert)
    
    def capColor(self, angle):
        return angle * self.invert
    
    def clampColor(self, pos):
        return pos * self.invert
