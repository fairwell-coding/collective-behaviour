from typing import Tuple, Dict, List
from fuzzylogic.classes import Rule

from environment import Environment
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

        self.coordinates = position  # current position P_n
        self.angle = angle  # direction X_in
        self.velocity = velocity  # movement speed V_n

    def __local_obstacle_avoiding_behavior(self, nearest_object_distances: Dict[str, float]) -> Tuple[float, float]:
        """ Implements the first behavior of category I pedestrians, i.e. "The Local Obstacle-Avoiding Behavior", of the paper.
        :param nearest_object_distances: dict of distances from the nearest object within field of vision, valid keys are 'l', 'fl', 'f', 'fr' and 'r' as described in the paper
        :return: tuple of current direction/angle and velocity/movement speed based on the corresponding rules
        """

        distances = self.domains.distances
        direction = self.domains.fov
        velocity = self.domains.velocity

        rules_direction = self.__create_local_obstacle_avoiding_behavior_rules(True, direction, velocity, distances)
        rules_velocity = self.__create_local_obstacle_avoiding_behavior_rules(False, direction, velocity, distances)

        values = {distances['l']: nearest_object_distances['l'] if 'l' in nearest_object_distances else Environment.dmax,
                  distances['fl']: nearest_object_distances['fl'] if 'fl' in nearest_object_distances else Environment.dmax,
                  distances['f']: nearest_object_distances['f'] if 'f' in nearest_object_distances else Environment.dmax,
                  distances['fr']: nearest_object_distances['fr'] if 'fr' in nearest_object_distances else Environment.dmax,
                  distances['r']: nearest_object_distances['r'] if 'r' in nearest_object_distances else Environment.dmax}

        return rules_direction(values), rules_velocity(values)

    @staticmethod
    def __create_local_obstacle_avoiding_behavior_rules(is_direction: bool, direction, velocity, distances):
        """ table I - the local obstacle avoiding behavior, i.e. 2^5 = 32 rules in total for all viewing angles
        :param is_direction: used to avoid duplicate code
        :param direction: orientation of pedestrian
        :param velocity: current pedestrian movement speed
        :param distances: distances to the nearest objects within the five angled areas within the field of view of a pedestrian
        :return: constructed rules
        """

        return Rule({(distances['l'].near, distances['fl'].near, distances['f'].near, distances['fr'].near, distances['r'].near): direction.zero if is_direction else velocity.stop,  # rule no. 1
                     (distances['l'].near, distances['fl'].near, distances['f'].near, distances['fr'].near, distances['r'].far): direction.large_pos if is_direction else velocity.slow,  # rule no. 2
                     (distances['l'].near, distances['fl'].near, distances['f'].near, distances['fr'].far, distances['r'].near): direction.small_pos if is_direction else velocity.slow,  # rule no. 3
                     (distances['l'].near, distances['fl'].near, distances['f'].near, distances['fr'].far, distances['r'].far): direction.large_pos if is_direction else velocity.slow,  # rule no. 4
                     (distances['l'].near, distances['fl'].near, distances['f'].far, distances['fr'].near, distances['r'].near): direction.zero if is_direction else velocity.fast,  # rule no. 5
                     (distances['l'].near, distances['fl'].near, distances['f'].far, distances['fr'].near, distances['r'].far): direction.zero if is_direction else velocity.fast,  # rule no. 6
                     (distances['l'].near, distances['fl'].near, distances['f'].far, distances['fr'].far, distances['r'].near): direction.zero if is_direction else velocity.fast,  # rule no. 7
                     (distances['l'].near, distances['fl'].near, distances['f'].far, distances['fr'].far, distances['r'].far): direction.zero if is_direction else velocity.fast,  # rule no. 8
                     (distances['l'].near, distances['fl'].far, distances['f'].near, distances['fr'].near, distances['r'].near): direction.small_neg if is_direction else velocity.slow,  # rule no. 9
                     (distances['l'].near, distances['fl'].far, distances['f'].near, distances['fr'].near, distances['r'].far): direction.small_neg if is_direction else velocity.slow,  # rule no. 10
                     (distances['l'].near, distances['fl'].far, distances['f'].near, distances['fr'].far, distances['r'].near): direction.small_neg if is_direction else velocity.slow,  # rule no. 11
                     (distances['l'].near, distances['fl'].far, distances['f'].near, distances['fr'].far, distances['r'].far): direction.large_pos if is_direction else velocity.slow,  # rule no. 12
                     (distances['l'].near, distances['fl'].far, distances['f'].far, distances['fr'].near, distances['r'].near): direction.zero if is_direction else velocity.fast,  # rule no. 13
                     (distances['l'].near, distances['fl'].far, distances['f'].far, distances['fr'].near, distances['r'].far): direction.zero if is_direction else velocity.fast,  # rule no. 14
                     (distances['l'].near, distances['fl'].far, distances['f'].far, distances['fr'].far, distances['r'].near): direction.zero if is_direction else velocity.fast,  # rule no. 15
                     (distances['l'].near, distances['fl'].far, distances['f'].far, distances['fr'].far, distances['r'].far): direction.zero if is_direction else velocity.fast,  # rule no. 16
                     (distances['l'].far, distances['fl'].near, distances['f'].near, distances['fr'].near, distances['r'].near): direction.large_neg if is_direction else velocity.slow,  # rule no. 17
                     (distances['l'].far, distances['fl'].near, distances['f'].near, distances['fr'].near, distances['r'].far): direction.large_pos if is_direction else velocity.slow,  # rule no. 18
                     (distances['l'].far, distances['fl'].near, distances['f'].near, distances['fr'].far, distances['r'].near): direction.small_pos if is_direction else velocity.slow,  # rule no. 19
                     (distances['l'].far, distances['fl'].near, distances['f'].near, distances['fr'].far, distances['r'].far): direction.large_pos if is_direction else velocity.slow,  # rule no. 20
                     (distances['l'].far, distances['fl'].near, distances['f'].far, distances['fr'].near, distances['r'].near): direction.zero if is_direction else velocity.fast,  # rule no. 21
                     (distances['l'].far, distances['fl'].near, distances['f'].far, distances['fr'].near, distances['r'].far): direction.zero if is_direction else velocity.fast,  # rule no. 22
                     (distances['l'].far, distances['fl'].near, distances['f'].far, distances['fr'].far, distances['r'].near): direction.zero if is_direction else velocity.fast,  # rule no. 23
                     (distances['l'].far, distances['fl'].near, distances['f'].far, distances['fr'].far, distances['r'].far): direction.zero if is_direction else velocity.fast,  # rule no. 24
                     (distances['l'].far, distances['fl'].far, distances['f'].near, distances['fr'].near, distances['r'].near): direction.large_neg if is_direction else velocity.slow,  # rule no. 25
                     (distances['l'].far, distances['fl'].far, distances['f'].near, distances['fr'].near, distances['r'].far): direction.large_neg if is_direction else velocity.slow,  # rule no. 26
                     (distances['l'].far, distances['fl'].far, distances['f'].near, distances['fr'].far, distances['r'].near): direction.large_neg if is_direction else velocity.slow,  # rule no. 27
                     (distances['l'].far, distances['fl'].far, distances['f'].near, distances['fr'].far, distances['r'].far): direction.large_pos if is_direction else velocity.slow,  # rule no. 28
                     (distances['l'].far, distances['fl'].far, distances['f'].far, distances['fr'].near, distances['r'].near): direction.zero if is_direction else velocity.fast,  # rule no. 29
                     (distances['l'].far, distances['fl'].far, distances['f'].far, distances['fr'].near, distances['r'].far): direction.zero if is_direction else velocity.fast,  # rule no. 30
                     (distances['l'].far, distances['fl'].far, distances['f'].far, distances['fr'].far, distances['r'].near): direction.zero if is_direction else velocity.fast,  # rule no. 31
                     (distances['l'].far, distances['fl'].far, distances['f'].far, distances['fr'].far, distances['r'].far): direction.zero if is_direction else velocity.fast,  # rule no. 32
                     })

    def __regional_path_searching_behavior(self):
        pass

    def __global_goal_seeking_behavior(self, angle: float, distance: float) -> Tuple[float, float]:
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

        return np.linalg.norm(np.asarray(self.coordinates) - np.asarray(self.goal.coordinates))

    def __calculate_angle_from_goal(self):
        """ Calculates angle between pedestrian and goal. Positive angles denote that the goal is on the right side of the pedestrian. Angles are calculated in degrees.
        :return: angle(pedestrian, goal) in degrees
        """

        p = np.asarray(self.coordinates)  # pedestrian
        x = np.asarray([self.coordinates[0] + 5, self.coordinates[1]])  # x-axis
        g = np.asarray(self.goal.coordinates)  # goal

        px = x - p
        pg = g - p

        alpha = np.degrees(np.arccos(np.dot(px, pg) / (np.linalg.norm(px) * np.linalg.norm(pg))))  # angle between goal, pedestrian and x-axis (our angle origin)

        # Correct calculated goal angle for 3rd and 4th quadrant
        if self.goal.coordinates[1] < self.coordinates[1]:
            alpha = 360 - alpha

        goal_angle = self.angle - alpha

        # Pushforward measure for angle, i.e. alpha is element of (-180, 180)
        if goal_angle < -180:
            goal_angle += 360

        if goal_angle > 180:
            goal_angle -= 360

        # Only allow final goal angles up to +/- 179 degrees to avoid goal seeker rules equalizing each other out when 180 degrees are reached
        if goal_angle < -179:
            goal_angle = -179

        if goal_angle > 179:
            goal_angle = 179

        return goal_angle

    def update(self):
        (angle_1, velocity_1) = self.__local_obstacle_avoiding_behavior({'l': 3.41})  # TODO: replace hardcoded distance with the nearest object distance from object detection algorithm
        (angle_3, velocity_3) = self.__global_goal_seeking_behavior(self.__calculate_angle_from_goal(), self.__calculate_distance_from_goal())
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
