# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 15:44:36 2022

@author: clive
"""

import pandas as pd
import numpy as np


class v_lut:
    
    def __init__(self, file, v_eq=2.8, v_min=2.5, v_max=2.9):
        self.df = pd.read_csv(file)
        
        self.v_min = v_min
        self.v_max = v_max
        self.v_eq = v_eq
        
        self.v_min_delta = v_min - v_eq
        self.v_max_delta = v_max - v_eq
        
        self.v_min_duty = self.look_up(v_min)
        self.v_max_duty = self.look_up(v_max)
        
        
    def vel2duty(self, vel):
        
        if (vel <= self.v_min_delta):
            ret_duty = self.v_min_duty
            
        elif (vel >= self.v_max_delta):
            ret_duty = self.v_max_duty
            
        else:
            ret_duty = self.look_up(vel + self.v_eq)
        
        return ret_duty 
    
    def look_up(self, vel):
        return self.df["duty_cycle"].iloc[np.max(np.where(self.df["air_vel_avg(m/s)"].le(vel)))]






# if (__name__ == "__main__"):
    
    # obj_lut = v_lut("../experiment/code/fan_charac/datasets/v_look_up_table.csv", 
                    # v_min = 2.251881,
                    # v_max = 3.490198)
    
