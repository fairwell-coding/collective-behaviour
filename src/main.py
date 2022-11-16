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

    def add_pedestrian(self, position: Tuple[float, float], angle: float, velocity: float) -> None:
        self.pedestrians.append(Pedestrian(self, position, angle, velocity))

    def add_obstacle(self, vertices: List[Tuple[float, float]]) -> None:
        self.obstacles.append(Obstacle(vertices))

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
    def __init__(self, simulation: Simulation, position: Tuple[float, float], angle=0.0, velocity=0.0) -> None:
        self.simulation = simulation
        self.domains = self.simulation.get_domains()

        self.position = position  # current position P_n
        self.angle = angle  # direction X_in
        self.velocity = velocity  # movement speed V_n

    def __local_obstacle_avoiding_behavior(self):
        pass

    def __regional_path_searching_behavior(self):
        pass

    def __goal_seeking_behavior(self, angle: float, distance: float) -> Tuple[float, float]:
        """ Implements the third behavior of category I pedestrians, i.e. "The Goal-Seeking Behavior", of the paper.
        :param angle: angle from goal
        :param distance: distance to goal
        :return: tuple of current direction/angle and velocity/movement speed based on the corresponding rules
        """

        goal_angle = self.domains.goal_angle
        goal_distance = self.domains.goal_distance
        direction = self.domains.direction
        velocity = self.domains.velocity

        rule_1 = Rule({(goal_angle.large_pos, goal_distance.near): (direction.large_neg, velocity.stop)})

        # table II - the goal seeking behavior
        rules = Rule({(goal_angle.large_pos, goal_distance.near): (direction.large_neg, velocity.stop),  # rule no. 1
                      (goal_angle.large_pos, goal_distance.far): (direction.large_neg, velocity.slow),  # rule no. 2
                      (goal_angle.small_pos, goal_distance.near): (direction.small_neg, velocity.slow),  # rule no. 3
                      (goal_angle.small_pos, goal_distance.far): (direction.small_neg, velocity.slow),  # rule no. 4
                      (goal_angle.zero, goal_distance.near): (direction.zero, velocity.fast),  # rule no. 5
                      (goal_angle.zero, goal_distance.far): (direction.zero, velocity.fast),  # rule no. 6
                      (goal_angle.small_neg, goal_distance.near): (direction.small_pos, velocity.slow),  # rule no. 7
                      (goal_angle.small_neg, goal_distance.far): (direction.small_pos, velocity.slow),  # rule no. 8
                      (goal_angle.large_neg, goal_distance.near): (direction.large_pos, velocity.stop),  # rule no. 9
                      (goal_angle.large_neg, goal_distance.far): (direction.large_pos, velocity.slow),  # rule no. 10
                      })

        values = {goal_angle: angle, goal_distance: distance}

        return rules(values)

    def update(self):
        # TODO: call rule 1 of pedestrian category I rules for first viable simulation
        (angle_3, velocity_3) = self.__goal_seeking_behavior(self.angle, self.velocity)  # TODO: refactor behavior out of pedestrian class
        # TODO: call rule 4 of pedestrian category I rules for first viable simulation


if __name__ == '__main__':
    sim = Simulation(FuzzyDomains())
    sim.add_pedestrian((1.31, 2.09), 0.0, 0.0)
    sim.add_obstacle([(1.0, 4.0), (4.0, 4.0), (4.0, 6.0), (1.0, 6.0)])
    sim.run()
