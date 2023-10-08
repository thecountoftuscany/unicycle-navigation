import numpy as np
import pygame
from pygame.locals import *
from robot import *
from sensor import *
import random

# Screen
screen_width = 640; screen_height = 480
screen = pygame.display.set_mode([screen_width, screen_height], DOUBLEBUF)

# Obstacles
num_circ_obsts = 8; obst_min_radius = 10; obst_max_radius = 50  # for circular obstacles

def create_circular_obsts(num):
    radius = []; circ_x = []; circ_y = []
    for i in range(num):
        radius.append(random.randint(obst_min_radius, obst_max_radius))
        circ_x.append(random.randint(radius[i], screen_width - radius[i]))
        circ_y.append(random.randint(radius[i], screen_height - radius[i]))
    return [radius, circ_x, circ_y]

def draw_circular_obsts(radius, circ_x, circ_y):
    for i in range(num_circ_obsts):
        pygame.draw.circle(screen, (0, 0, 255), (circ_x[i], circ_y[i]), radius[i], 0)

def main():
    # PyGame inits
    pygame.init()
    pygame.display.set_caption('Unicycle robot')
    clock = pygame.time.Clock()
    ticks = pygame.time.get_ticks()
    frames = 0

    # Robot
    robot_x = 100; robot_y = 100; robot_phi = 0; robot_l = 15; robot_b = 6  # Initial position
    skirt_r = 30   # Sensor skirt radius
    num_sensors = 8 # Number of distance sensors
    goalX = np.array([600, 400])    # goal position

    data = {"screen":screen, "goalX":goalX, "vmax":0.5, "gtg_scaling":0.0001, "K_p":0.02, "ao_scaling":0.00005}

    # Create obstacles
    [radius, circ_x, circ_y] = create_circular_obsts(num_circ_obsts)

    # PyGame loop
    while(1):
        # To exit
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break
        screen.fill((50, 55, 60))   # background

        # Draw robot, sensor skirt, obstacles and goal
        bot = robot(robot_x, robot_y, robot_phi, robot_l, robot_b, data)
        sensors = sensor(num_sensors, skirt_r, bot.x, bot.y, bot.phi, data)
        draw_circular_obsts(radius, circ_x, circ_y)
        pygame.draw.circle(screen, (0,255,0), goalX, 8, 0)  # Draw goal
        sensors.show() # Draw the sensors
        bot.show()    # Draw the robot

        # Check if obstacles are in sensor skirt
        close_obst = []; dist = []
        for i in range(num_circ_obsts):
            distance = math.sqrt((circ_x[i] - robot_x)**2 + (circ_y[i] - robot_y)**2)
            if( distance <= (skirt_r + radius[i])):
                close_obst.append([circ_x[i], circ_y[i], radius[i]])
                dist.append(distance)
        # Go to goal
        if(len(close_obst) == 0):           # No obstacle in sensor skirt
            [v, omega] = bot.go_to_goal()   # output from controller go_to_goal()
        # Paranoid behavior - run away from obstacle
        else:
            closest_obj = dist.index(min(dist)) # gives the index of the closest object
            obstX = np.array([circ_x[closest_obj], circ_y[closest_obj]])
            [v, omega] = bot.avoid_obst(obstX)

        # Update robot position and orientation as per control input
        robot_x += v*math.cos(robot_phi); robot_y+= v*math.sin(robot_phi); robot_phi += omega

        # FPS. Print if required
        clock.tick(300)     # To limit fps, controls speed of the animation
        fps = (frames*1000)/(pygame.time.get_ticks() - ticks)   # calculate current fps

        # Update PyGame display
        pygame.display.flip()
        frames+=1

if(__name__ == '__main__'):
    main()
