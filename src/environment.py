import numpy as np
import pygame

class Environment:
    reach_max = 0.75 # maximum reach of assailants
    vmax = 13.58  # avg running speed of male = 3.58m/s #EDIT: Changed to 33.58 for testing purpuses
    fov = 170  # field of view of a pedestrian measured in degrees
    dmax = 5  # pedestrians may only see up to a radius of 5meters
    gmax = 100  # maximum possible goal distance
    g_angle_max = 360  # maximum range of goal angle
    neg_energy_max = 1.0 # maximum negative energy
    k_w = 0.4 # weighting factor for calculating negative energy
    occupied_g_angle_max = 40  # maximum range of occupied goal angle
    oi_max = 1.0  # maximum range of obstacle impact
    cr_max = 1.0  # maximum range of collision risk
    fov_ray_angles = [0,40,30,30,30,40]
    goal_tolerance = 3
    zone_names = ['l', 'fl', 'f', 'fr', 'r']
    
    fov_ray_angles_rad = np.add.accumulate(np.radians(fov_ray_angles))
    fov_ray_angles_rad -= fov_ray_angles_rad[-1] / 2
    
    pygame.init()
    color_black = pygame.Color('#000000')
    color_green = pygame.Color('#00FF00')
    color_red = pygame.Color('#FF0000')
    font = pygame.font.SysFont(None, 32)