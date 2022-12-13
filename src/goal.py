from typing import TYPE_CHECKING, Tuple

import numpy as np
from environment import Environment
import pygame


class Goal(pygame.sprite.Sprite):

    def __init__(self, coordinates: Tuple[float, float]) -> None:
        super().__init__()
        self.coordinates = np.array(coordinates)

    def get_coordinates(self) -> Tuple[float, float]:
        return self.coordinates

    def draw(self, screen: pygame.Surface, scale: float):
        coordinates_scaled = self.coordinates * scale
        pygame.draw.circle(screen, Environment.color_red, coordinates_scaled, 5)