import numpy as np
import pygame
import math

class sensor:

    def __init__(self, num, skirt_r, x, y, phi, data):
        self.num = num
        self.skirt_r = skirt_r
        self.x = x
        self.y = y
        self.phi = phi
        self.X = np.array([self.x, self.y])
        self.data = data

    def show(self):
        for i in range(self.num):
            pygame.draw.line(self.data["screen"], (255,255,0), (self.x,self.y), (self.x+self.skirt_r*math.cos(self.phi+(math.pi/self.num)+(2*math.pi*i/self.num)), self.y+self.skirt_r*math.sin(self.phi+(math.pi/self.num)+(2*math.pi*i/self.num))))

