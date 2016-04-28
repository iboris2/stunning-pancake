class Playground(object):
    def __init__(self, color='purple'):
        self.color = color
        
        if self.color == 'purple':
            invert = - 1
        else:
            invert = 1
        
        self.all_seashell = []
        self.all_seashell.append([1250, 1300 * invert]) # 0
        self.all_seashell.append([1550, 1300 * invert]) #1
       
        self.all_seashell.append([1250, 800 * invert]) #2
        self.all_seashell.append([1550, 800 * invert]) #3
        self.all_seashell.append([1850, 800 * invert]) #4
       
        self.all_seashell.append([1450, 600 * invert]) #5
        self.all_seashell.append([1650, 300 * invert]) #6
       
        self.all_seashell.append([1550, 0 * invert]) #7
        self.all_seashell.append([1850, 0 * invert]) #8
       
        self.all_seashell.append([1650, -300 * invert]) #9
        
        self.seasheel_collect = [850, 1200 * invert]
        
        # door
        self.hut = [
                    [0, 1200 * invert],
                    [0, 900 * invert] 
                    ]
        #fish spot
        self._fish_area = [ 
                          [2000, 1000 * invert],                    
                          [2000, 600 * invert]
                          ]
        e = 80
        self.fish_spot = [
                          [2000, (800 + e) * invert],                    
                          [2000, (800 - e) * invert]
                          ]
        self._depose_area = [ 
                          [2000, 550 * invert],                    
                          [2000, 0 * invert]
                          ]
        ecart_bord = 180
        ecart_spot = 150
        self.depose_spot = [
                          [2000, (550 - ecart_bord) * invert],                    
                          [2000, (550 - ecart_bord - ecart_spot) * invert]
                          ]
        
        #sand
        self.sand = [
                     [900, 850 * invert],
                     [58, 678 * invert],
                     [58 * 2, 0]
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
        
