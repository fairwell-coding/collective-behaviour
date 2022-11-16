from typing import List, Tuple
import numpy as np
import pygame

RESOLUTION = 100
THICKNESS = 0.3
FRAMERATE = 60
cone_color = pygame.Color('#CCCCCC')
circle_color = pygame.Color('#212121')
wall_color = pygame.Color('#000000')


class Person(pygame.sprite.Sprite):
    def __init__(self, start_point: Tuple[float, float],speed = 2) -> None:
        super().__init__()
        
        self._position = np.array(start_point, dtype=float)
        self._angle = 0
        self._speed = speed
        
        self._cone_radius = 5 * RESOLUTION
        self._circle_radius = 0.75 * RESOLUTION
        self._surface_size = 2 * 5 * RESOLUTION

        surface = pygame.Surface((self._surface_size, self._surface_size), pygame.SRCALPHA)
        #surface.fill('#FFFFFF')

        angles = np.arange(180 + 5 - 1, 180 + 175 + 2) / 180 * np.pi
        x = self._cone_radius + self._cone_radius * np.cos(angles)
        y = self._cone_radius + self._cone_radius * np.sin(angles)
        points = np.vstack([x, y]).T
        points[[0, -1], :] = [self._cone_radius, self._cone_radius]

        #surface.set_colorkey((0,0,0))
        pygame.draw.polygon(surface, cone_color, points)
        pygame.draw.circle(surface, circle_color, (self._cone_radius, self._cone_radius), self._circle_radius)
        
        self._original_surface = surface
        

    def draw(self, screen: pygame.Surface, scale: float):
        surface = self._original_surface
        surface = pygame.transform.rotate(surface, self._angle)
        padding = (surface.get_rect()[2] - self._surface_size) // 2
        surface = surface.subsurface(padding, padding, self._surface_size, self._surface_size)
        
        surface = pygame.transform.smoothscale(surface, (self._surface_size / RESOLUTION * scale, self._surface_size / RESOLUTION * scale))
        #position = (self._position[0] - (self._cone_radius * scale), self._position[1] - (self._cone_radius * scale))
        position = self._position * scale - (self._cone_radius / RESOLUTION * scale)
        screen.blit(surface, position)

    def updateFromKeyboard(self, pressed_keys, scale):
        rad = np.radians(self._angle)
        move = self._speed / FRAMERATE * np.array([np.sin(rad), np.cos(rad)]) * scale
        if pressed_keys[pygame.K_UP] or pressed_keys[pygame.K_w]:
            self._position -= move
        if pressed_keys[pygame.K_DOWN] or pressed_keys[pygame.K_s]:
            self._position += move
        if pressed_keys[pygame.K_LEFT] or pressed_keys[pygame.K_a]:
            self._angle += 5
        if pressed_keys[pygame.K_RIGHT] or pressed_keys[pygame.K_d]:
            self._angle -= 5
            
            
class Obstacle(pygame.sprite.Sprite):
    def __init__(self, vertices: List[Tuple[float,float]]) -> None:
        super().__init__()
        self._vertices = np.array(vertices)
        
    def draw(self,screen: pygame.Surface, scale: float):
        vertices = self._vertices * scale
        pygame.draw.polygon(screen, wall_color, vertices, int(np.ceil(THICKNESS * scale)))


""" class Simulation:
    def __init__(self, screen_size: Tuple[int, int], scale: float) -> None:
        pygame.init()
        self._screen_size = screen_size
        self._scale = scale
        
    def run(self):
        screen = pygame.display.set_mode(self._screen_size)
        
        person 
        
        running = True
        while running:

            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False

            pressed_keys = pygame.key.get_pressed()
            person.updatePos(pressed_keys)
            
            screen.fill((255, 255, 255))
            screen.blit(person.surface, person.rect)

            pygame.display.flip()
            clock = pygame.time.Clock()
            clock.tick(60) """

screen = pygame.display.set_mode([840, 540])

person = Person((24, 16))
obstacles: List[Obstacle] = []
obstacles.append(Obstacle([[140.0, 70.0], [130.0, 60.0], [123.10553461267196, 56.82498524140448], [107.43477909864454, 68.32529775573109], [84.5605311305884, 62.2591988470753], [56.50482367805545, 65.54500242259718], [38.43290401268513, 62.511952968269284], [22.13026319567274, 68.45167481632808], [15.179524862838, 62.891084150060266], [22.888525559254713, 47.220328636032804], [48.03756061805677, 44.1872791817049], [75.84051394939573, 38.24755733364611], [102.50607373536172, 40.143213242601036], [118.80871455237411, 34.07711433394525], [140.0, 33.31885197036327], [140.0, 70.0]]))
obstacles.append(Obstacle([[0.6461628941835449, 33.44522903096027], [29.081001528507482, 31.802327243199322], [55.3674301326825, 22.703178880215624], [78.87356340372361, 21.692162395439667], [110.0, 20.0], [107.94028734103252, 14.10953875961992], [96.56635188730294, 11.834751668873995], [71.92282507088888, 13.856784638425923], [53.21902010253358, 16.384325850365844], [24.025919104627672, 18.659112941111758], [-1.734723475976807e-17, 12.34025991126198]]))

running = True
while running:

    for event in pygame.event.get():
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False

    pressed_keys = pygame.key.get_pressed()
    person.updateFromKeyboard(pressed_keys, 6)

    screen.fill((255, 255, 255))
    for o in obstacles:
        o.draw(screen, 6)
    person.draw(screen, 6)

    pygame.display.flip()
    clock = pygame.time.Clock()
    clock.tick(FRAMERATE)
