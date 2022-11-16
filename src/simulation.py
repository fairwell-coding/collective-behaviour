from typing import Tuple, List
from fuzzy_domains import FuzzyDomains
from goal import Goal
from obstacle import Obstacle


class Simulation:
    def __init__(self, fuzzy_domains: FuzzyDomains, goal: Goal, tick_rate=60) -> None:
        self.fuzzy_domains = fuzzy_domains
        self.goal = goal
        self.pedestrians = []
        self.obstacles = []
        self.tick_rate = tick_rate

    def get_domains(self):
        return self.fuzzy_domains

    def get_goal(self):
        return self.goal

    def add_pedestrian(self, position: Tuple[float, float], angle: float, velocity: float) -> None:
        from pedestrian import Pedestrian
        self.pedestrians.append(Pedestrian(self, position, angle, velocity))

    def add_obstacle(self, vertices: List[Tuple[float, float]]) -> None:
        self.obstacles.append(Obstacle(vertices))

    def run(self):
        for pedestrian in self.pedestrians:
            pedestrian.update()

        # TODO: extend simulation
