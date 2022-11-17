from typing import Tuple
from fuzzylogic.classes import Rule
from simulation import Simulation
import numpy as np


class Pedestrian:
    def __init__(self, simulation: Simulation, position: Tuple[float, float], angle: float, velocity=0.0) -> None:
        if angle < 0.0 or angle > 359.0:
            raise IOError("Valid direction angles for a pedestrian are within 0 and 359 degrees.")  # 360 would be equal to 0 and thus result in issues with the fuzzy logic rule application (
            # turning left and right at the same time)

        self.simulation = simulation
        self.domains = self.simulation.get_domains()
        self.goal = self.simulation.get_goal()

        self.position = position  # current position P_n
        self.angle = angle  # direction X_in
        self.velocity = velocity  # movement speed V_n

    def __local_obstacle_avoiding_behavior(self, distance: float) -> Tuple[float, float]:
        """ Implements the first behavior of category I pedestrians, i.e. "The Local Obstacle-Avoiding Behavior", of the paper.
        :param distance: distance from the nearest object within field of vision
        :return: tuple of current direction/angle and velocity/movement speed based on the corresponding rules
        """

        dist = self.domains.distance
        direction = self.domains.direction
        velocity = self.domains.velocity

        # table I - the local obstacle-avoiding behavior
        rules = Rule({dist.near: (direction.large_neg, velocity.stop),  # rule no. 1
                      # TODO: define near and far for all 5 viewing angles, then add all 2^5 = 32 rules
                      })

        values = {dist: distance}

        return rules(values)  # TODO: fix rule evaluation bug

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

        return rules(values)  # TODO: fix rule evaluation bug

    def __calculate_distance_from_goal(self):
        """ Calculates Euclidean distance between goal and pedestrian.
        :return: Euclidean distance from goal
        """

        return np.linalg.norm(np.asarray(self.position) - np.asarray(self.goal.coordinates))

    def __calculate_angle_from_goal(self):
        return 0.0  # TODO: calculate angle

    def update(self):
        (angle_1, velocity_1) = self.__local_obstacle_avoiding_behavior(3.76)  # TODO: replace hardcoded distance with the nearest object distance
        (angle_3, velocity_3) = self.__goal_seeking_behavior(self.__calculate_angle_from_goal(), self.__calculate_distance_from_goal())
        # TODO: calculate weighted balance of above metrics
