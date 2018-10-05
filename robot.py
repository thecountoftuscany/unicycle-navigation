import numpy as np
import pygame
import math

class robot:

    def __init__(self, init_x, init_y, init_phi, robot_l, robot_b, data):
        self.x = init_x
        self.y = init_y
        self.phi = init_phi
        self.l = robot_l # Robot length is 2*l
        self.b = robot_b # Robot breadth is 2*b
        self.X = np.array([self.x, self.y])
        self.data = data
        self.tip = [self.x + self.l * math.cos(self.phi), self.y + self.l * math.sin(self.phi)]
        self.bottom = [self.x - self.l * math.cos(self.phi), self.y - self.l * math.sin(self.phi)]
        self.bottom_l = [self.bottom[0] - self.b * math.sin(self.phi), self.bottom[1] + self.b * math.cos(self.phi)]
        self.bottom_r = [self.bottom[0] + self.b * math.sin(self.phi), self.bottom[1] - self.b * math.cos(self.phi)]

    def show(self):
        pygame.draw.polygon(self.data["screen"], (255,0,0), [self.tip, self.bottom_l, self.bottom_r], 0)

    def go_to_goal(self):
        e = self.data["goalX"] - self.X     # error in position
        K = self.data["vmax"] * (1 - np.exp(- self.data["gtg_scaling"] * np.linalg.norm(e)**2)) / np.linalg.norm(e)     # Scaling for velocity
        v = np.linalg.norm(K * e)   # Velocity decreases as bot gets closer to goal
        phi_d = math.atan2(e[1], e[0])  # Desired heading
        omega = self.data["K_p"]*math.atan2(math.sin(phi_d - self.phi), math.cos(phi_d - self.phi))     # Only P part of a PID controller to give omega as per desired heading
        return [v, omega]

    def avoid_obst(self, obstX):
        e = obstX - self.X     # error in position
        K = self.data["vmax"] * (1 - np.exp(- self.data["ao_scaling"] * np.linalg.norm(e)**2)) / np.linalg.norm(e)      # Scaling for velocity
        v = np.linalg.norm(K * e)   # Velocity decreases as bot gets closer to obstacle
        phi_d = -math.atan2(e[1], e[0]) # Desired heading
        omega = self.data["K_p"]*math.atan2(math.sin(phi_d - self.phi), math.cos(phi_d - self.phi))     # Only P part of a PID controller to give omega as per desired heading
        return [v, omega]
