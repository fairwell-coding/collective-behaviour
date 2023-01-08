from typing import Tuple, List
from fuzzy_domains import FuzzyDomains
from goal import Goal
from obstacle import Obstacle
from pedestrian import Pedestrian
from assailant import Assailant
import pygame


class Simulation:
    def __init__(self, fuzzy_domains: FuzzyDomains, goal: Goal, scale = 5, tick_rate=60) -> None:
        self.fuzzy_domains = fuzzy_domains
        self.goal = goal
        self.pedestrians: List[Pedestrian] = []
        self.assailants: List[Assailant] = []
        self.obstacles: List[Obstacle] = []
        
        self.tick_rate = tick_rate
        self.scale = scale
        self.screen = pygame.display.set_mode([840,540])

    def get_domains(self):
        return self.fuzzy_domains

    def get_goal(self):
        return self.goal

    def add_pedestrian(self, position: Tuple[float, float], angle: float, velocity: float) -> None:
        self.pedestrians.append(Pedestrian(self, position, angle, velocity))

    def add_assailant(self, position: Tuple[float, float], angle: float, velocity: float) -> None:
        self.assailants.append(Assailant(self, position, angle, velocity))

    def add_obstacle(self, vertices: List[Tuple[float, float]]) -> None:
        self.obstacles.append(Obstacle(vertices, self))

    def get_pedestrians(self):
        return self.pedestrians

    def get_assailants(self):
        return self.assailants

    def get_obstacles(self):
        return self.obstacles

    def run(self):
        running = True
        clock = pygame.time.Clock()
        #manual_pedestrian = Pedestrian(self,(0,0),90)
        
        while running:

            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE \
                or event.type == pygame.QUIT:
                    running = False

            for pedestrian in self.pedestrians:
                pedestrian.update()

            for assailant in self.assailants:
                assailant.update()
            
            self.screen.fill((255,255,255))
            #manual_pedestrian.update_from_keyboard(pygame.key.get_pressed())
            
            for obstacle in self.obstacles:
                obstacle.draw()
                
            for pedestrian in self.pedestrians:
                pedestrian.draw()

            for assailant in self.assailants:
                assailant.draw()
                
            self.goal.draw(self.screen, self.scale)
            
            #manual_pedestrian.draw()
            
            pygame.display.flip()
            clock.tick(self.tick_rate)
