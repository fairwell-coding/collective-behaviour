from typing import Tuple, Dict, List
from fuzzylogic.classes import Rule
from simulation import Simulation
import numpy as np


class Pedestrian:
    def __init__(self, simulation: Simulation, position: Tuple[float, float], angle: float, velocity=0.0) -> None:
        if angle < 0.0 or angle > 359.0:
            raise IOError("Valid direction angles for a pedestrian are within 0 and 359 degrees.")  # 360 would be equal to 0 and thus result in issues with the fuzzy logic rule application, i.e.
            # turning left and right at the same time

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

        return 0.0, 3.4

        # dist = self.domains.distance
        # direction = self.domains.direction
        # velocity = self.domains.velocity
        #
        # # table I - the local obstacle-avoiding behavior
        # rules = Rule({dist.near: (direction.large_neg, velocity.stop),  # rule no. 1
        #               # TODO: define near and far for all 5 viewing angles, then add all 2^5 = 32 rules
        #               })
        #
        # values = {dist: distance}
        #
        # return rules(values)

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
        direction = self.domains.fov
        velocity = self.domains.velocity

        rules_direction = self.__create_goal_seeking_behavior_rules(True, direction, goal_angle, goal_distance, velocity)
        rules_velocity = self.__create_goal_seeking_behavior_rules(False, direction, goal_angle, goal_distance, velocity)
        values = {goal_angle: angle, goal_distance: distance}

        return rules_direction(values), rules_velocity(values)

    @staticmethod
    def __create_goal_seeking_behavior_rules(is_direction: bool, direction, goal_angle, goal_distance, velocity):
        """ table II - the goal seeking behavior
        :param direction: orientation of pedestrian
        :param goal_angle: angle towards goal
        :param goal_distance: distance to goal
        :param velocity: current pedestrian movement speed
        :return: constructed rules
        """

        return Rule({(goal_angle.large_pos, goal_distance.near): direction.large_neg if is_direction else velocity.stop,  # rule no. 1
                     (goal_angle.large_pos, goal_distance.far): direction.large_neg if is_direction else velocity.slow,  # rule no. 2
                     (goal_angle.small_pos, goal_distance.near): direction.small_neg if is_direction else velocity.slow,  # rule no. 3
                     (goal_angle.small_pos, goal_distance.far): direction.small_neg if is_direction else velocity.slow,  # rule no. 4
                     (goal_angle.zero, goal_distance.near): direction.zero if is_direction else velocity.fast,  # rule no. 5
                     (goal_angle.zero, goal_distance.far): direction.zero if is_direction else velocity.fast,  # rule no. 6
                     (goal_angle.small_neg, goal_distance.near): direction.small_pos if is_direction else velocity.slow,  # rule no. 7
                     (goal_angle.small_neg, goal_distance.far): direction.small_pos if is_direction else velocity.slow,  # rule no. 8
                     (goal_angle.large_neg, goal_distance.near): direction.large_pos if is_direction else velocity.stop,  # rule no. 9
                     (goal_angle.large_neg, goal_distance.far): direction.large_pos if is_direction else velocity.slow,  # rule no. 10
                     })

    def __calculate_distance_from_goal(self):
        """ Calculates Euclidean distance between goal and pedestrian.
        :return: Euclidean distance from goal
        """

        return np.linalg.norm(np.asarray(self.position) - np.asarray(self.goal.coordinates))

    def __calculate_angle_from_goal(self):
        """ Calculates angle between pedestrian and goal. Positive angles denote that the goal is on the right side of the pedestrian. Angles are calculated in degrees.
        :return: angle(pedestrian, goal) in degrees
        """

        p = np.asarray(self.position)  # pedestrian
        x = np.asarray([self.position[0] + 5, self.position[1]])  # x-axis
        g = np.asarray(self.goal.coordinates)  # goal

        px = x - p
        pg = g - p

        alpha = np.degrees(np.arccos(np.dot(px, pg) / (np.linalg.norm(px) * np.linalg.norm(pg))))  # angle between goal, pedestrian and x-axis (our angle origin)
        # TODO: 3rd and 4th quadrant should add 180 degrees or return negative results in order to be conform to pedestrian orientation

        return self.angle - alpha

    def update(self):
        (angle_1, velocity_1) = self.__local_obstacle_avoiding_behavior(3.76)  # TODO: replace hardcoded distance with the nearest object distance
        (angle_3, velocity_3) = self.__goal_seeking_behavior(self.__calculate_angle_from_goal(), self.__calculate_distance_from_goal())
        movement_speed, turning_angle = self.__integrate_multiple_behaviors([angle_1, angle_3], [velocity_1, velocity_3])
        print(f'Turning angle: {turning_angle}, movement speed: {movement_speed}')  # TODO: connect output values with visualization

    @staticmethod
    def __integrate_multiple_behaviors(angles: List[float], velocities: List[float]):
        """ Integrate multiple behaviors to determine the degree of effect of each behavior on the final result
        :param angles: angles calculated based on different fuzzy logic behaviors
        :param velocities: velocities calculated based on different fuzzy logic behaviors
        :return: weighted balance of turning angle and movement speed
        """

        turning_angle = np.mean(angles)
        movement_speed = np.mean(velocities)

        return movement_speed, turning_angle
