from typing import List, Tuple, TYPE_CHECKING
import pygame
import numpy as np
from environment import Environment

if TYPE_CHECKING:
    from simulation import Simulation


class Obstacle(pygame.sprite.Sprite):    
    def __init__(self, vertices: List[Tuple[float, float]], simulation: 'Simulation') -> None:
        super().__init__()
        self.vertices = np.array(vertices)
        self.simulation = simulation

    def get_vertices(self) -> List[Tuple[float, float]]:
        return self.vertices
    
    def draw(self):
        vertices_scaled = self.vertices * self.simulation.scale
        pygame.draw.lines(self.simulation.screen, Environment.color_black, False, vertices_scaled, 1)
