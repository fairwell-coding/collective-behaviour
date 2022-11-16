from typing import List, Tuple
from fuzzylogic.classes import Domain, Rule

from fuzzy_domains import FuzzyDomains


class Simulation:
    def __init__(self, fuzzy_domains: FuzzyDomains, tick_rate=60) -> None:
        self.fuzzy_domains = fuzzy_domains
        self.pedestrians = []
        self.obstacles = []
        self.tick_rate = tick_rate

    def get_domains(self):
        return self.fuzzy_domains

    def add_pedestrian(self, position: Tuple[float, float]):
        self.pedestrians.append(Pedestrian((position[0], position[1]), self))

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def run(self):
        for pedestrian in self.pedestrians:
            pedestrian.update()

        # TODO: extend simulation


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

    def __loc_obstacle_avoiding(self):  # Outputs speed and angle
        pass

    def __path_searching(self):
        pass

    def __goal_seeking_behavior(self, goal_angle: Domain, goal_distance: Domain):
        goal_angle = self.simulation.get_domains().get_goal_angle_domain()
        goal_distance = self.simulation.get_domains().get_goal_distance_domain()

        rules = Rule({(goal_angle.large_pos, goal_distance.near): ''  # rule no. 1
                      })

        # TODO: define rules from table II
        # TODO: apply R4 formula from paper here

        pass

    def update(self):
        self.__goal_seeking_behavior()
        # TODO: call rule 1 and 4 of pedestrian category I rules for first viable simulation


if __name__ == '__main__':
    sim = Simulation(FuzzyDomains())
    sim.add_pedestrian((1.31, 2.09))
    sim.run()
