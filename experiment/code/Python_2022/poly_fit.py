# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 21:29:56 2022

@author: clive
"""


class v_polyfit:
    
    def __init__(self, v_eq=2.8, v_min=2.5, v_max=2.9, p1=93.55, p2=-416.5, p3=505.1):
        
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        
        self.v_min = v_min
        self.v_max = v_max
        self.v_eq = v_eq
        
        self.v_min_delta = v_min - v_eq
        self.v_max_delta = v_max - v_eq
        
        self.v_min_duty = self.poly_eval(v_min)
        self.v_max_duty = self.poly_eval(v_max)
        
        
    def vel2duty(self, vel):
        
        if (vel <= self.v_min_delta):
            ret_duty = self.v_min_duty
            
        elif (vel >= self.v_max_delta):
            ret_duty = self.v_max_duty
            
        else:
            ret_duty = self.poly_eval(vel + self.v_eq)
        
        return ret_duty 
    
    def poly_eval(self, vel):
        return self.p1*vel**2 + self.p2*vel + self.p3






if (__name__ == "__main__"):
    obj_poly = v_polyfit()
    print(obj_poly.vel2duty(2.8 - 2.8))
