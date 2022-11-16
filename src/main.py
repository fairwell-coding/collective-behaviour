from typing import List, Tuple

from fuzzylogic.classes import Domain

from fuzzy_domains import FuzzyDomains


class Simulation:
    def __init__(self, tick_rate=60) -> None:
        self.pedestrian = []
        self.obstacles = []
        self.tick_rate = tick_rate

    def add_pedestrian(self, pedestrian):
        self.pedestrian.append(pedestrian)

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def update(self):
        pass


class Obstacle:
    def __init__(self, vertices: List[Tuple[float]]) -> None:
        self.vertices = vertices

    def get_vertices(self) -> List[Tuple[float]]:
        return self.vertices


class Pedestrian:
    def __init__(self, position: Tuple[float, float], simulation: Simulation) -> None:
        self.simulation = simulation
        self.position: Tuple[float, float] = position
        self.velocity: float = 0
        self.angle: float = 0

    def loc_obstacle_avoiding(self):  # Outputs speed and angle
        pass

    def path_searching(self):
        pass

    def goal_seeking_behavior(self, goal_angle: Domain, goal_distance: Domain):
        pass

    def update(self):
        pass
        # TODO: call all four pedestrian category I rules


if __name__ == '__main__':
    sim = Simulation()
    domains = FuzzyDomains()
