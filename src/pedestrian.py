from typing import Tuple, Dict, List, TYPE_CHECKING
from fuzzylogic.classes import Rule
from algebra_functions import angle_between, distance_between
from environment import Environment
from obstacle import Obstacle
from utils import Utils
import numpy as np
import pygame

if TYPE_CHECKING:
    from simulation import Simulation

class Pedestrian(pygame.sprite.Sprite):    
    def __init__(self, simulation: 'Simulation', position: Tuple[float, float], angle: float, velocity=0.0, manual = False) -> None:
        super().__init__()
        if angle < 0.0 or angle > 359.0:
            raise IOError("Valid direction angles for a pedestrian are within 0 and 359 degrees.")  # 360 would be equal to 0 and thus result in issues with the fuzzy logic rule application, i.e.
            # turning left and right at the same time

        self.simulation = simulation
        self.domains = self.simulation.get_domains()
        self.goal = self.simulation.get_goal()
        self.manual = manual

        self.coordinates = np.array(position, dtype=float)  # current position P_n
        self.angle = angle  # direction X_in
        self.velocity = velocity  # movement speed V_n
        
        self.__update_position((0,0),0)

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

    def __regional_path_searching_behavior(self, negative_energy_per_sector: Dict[str, float]) -> Tuple[float, float]:
        """ Implements the second behavior of category I pedestrians, i.e. "The Regional Path-Searching Behavior", of the paper.
        :param negative_energy_per_sector: dict of normalized negative energies from each sector within field of vision, valid keys are 'l', 'fl', 'f', 'fr' and 'r' as described in the paper
        :return: tuple of current direction/angle and velocity/movement speed based on the corresponding rules
        """

        negative_energies = self.domains.negative_energies
        direction = self.domains.fov
        velocity = self.domains.velocity

        rules_direction = self.__create_regional_path_searching_behavior_rules(True, direction, velocity, negative_energies)
        rules_velocity = self.__create_regional_path_searching_behavior_rules(False, direction, velocity, negative_energies)

        values = {negative_energies['l']: negative_energy_per_sector['l'] if 'l' in negative_energy_per_sector else Environment.neg_energy_max,
                  negative_energies['fl']: negative_energy_per_sector['fl'] if 'fl' in negative_energy_per_sector else Environment.neg_energy_max,
                  negative_energies['f']: negative_energy_per_sector['f'] if 'f' in negative_energy_per_sector else Environment.neg_energy_max,
                  negative_energies['fr']: negative_energy_per_sector['fr'] if 'fr' in negative_energy_per_sector else Environment.neg_energy_max,
                  negative_energies['r']: negative_energy_per_sector['r'] if 'r' in negative_energy_per_sector else Environment.neg_energy_max}

        return rules_direction(values), rules_velocity(values)

    @staticmethod
    def __create_regional_path_searching_behavior_rules(is_direction: bool, direction, velocity, negative_energies):
        """ The rule set of regional path-searching behavior is similar to table I - the local obstacle avoiding behavior, with distances replaced by normalized negative energies per each sector.
        :param is_direction: used to avoid duplicate code
        :param direction: orientation of pedestrian
        :param velocity: current pedestrian movement speed
        :param negative_energies: negative energies within the five angled areas within the field of view of a pedestrian
        :return: constructed rules
        """

        return Rule({(negative_energies['l'].high, negative_energies['fl'].high, negative_energies['f'].high, negative_energies['fr'].high, negative_energies['r'].high): direction.zero if is_direction else velocity.stop,  # rule no. 1
                     (negative_energies['l'].high, negative_energies['fl'].high, negative_energies['f'].high, negative_energies['fr'].high, negative_energies['r'].low): direction.large_pos if is_direction else velocity.slow,  # rule no. 2
                     (negative_energies['l'].high, negative_energies['fl'].high, negative_energies['f'].high, negative_energies['fr'].low, negative_energies['r'].high): direction.small_pos if is_direction else velocity.slow,  # rule no. 3
                     (negative_energies['l'].high, negative_energies['fl'].high, negative_energies['f'].high, negative_energies['fr'].low, negative_energies['r'].low): direction.large_pos if is_direction else velocity.slow,  # rule no. 4
                     (negative_energies['l'].high, negative_energies['fl'].high, negative_energies['f'].low, negative_energies['fr'].high, negative_energies['r'].high): direction.zero if is_direction else velocity.fast,  # rule no. 5
                     (negative_energies['l'].high, negative_energies['fl'].high, negative_energies['f'].low, negative_energies['fr'].high, negative_energies['r'].low): direction.zero if is_direction else velocity.fast,  # rule no. 6
                     (negative_energies['l'].high, negative_energies['fl'].high, negative_energies['f'].low, negative_energies['fr'].low, negative_energies['r'].high): direction.zero if is_direction else velocity.fast,  # rule no. 7
                     (negative_energies['l'].high, negative_energies['fl'].high, negative_energies['f'].low, negative_energies['fr'].low, negative_energies['r'].low): direction.zero if is_direction else velocity.fast,  # rule no. 8
                     (negative_energies['l'].high, negative_energies['fl'].low, negative_energies['f'].high, negative_energies['fr'].high, negative_energies['r'].high): direction.small_neg if is_direction else velocity.slow,  # rule no. 9
                     (negative_energies['l'].high, negative_energies['fl'].low, negative_energies['f'].high, negative_energies['fr'].high, negative_energies['r'].low): direction.small_neg if is_direction else velocity.slow,  # rule no. 10
                     (negative_energies['l'].high, negative_energies['fl'].low, negative_energies['f'].high, negative_energies['fr'].low, negative_energies['r'].high): direction.small_neg if is_direction else velocity.slow,  # rule no. 11
                     (negative_energies['l'].high, negative_energies['fl'].low, negative_energies['f'].high, negative_energies['fr'].low, negative_energies['r'].low): direction.large_pos if is_direction else velocity.slow,  # rule no. 12
                     (negative_energies['l'].high, negative_energies['fl'].low, negative_energies['f'].low, negative_energies['fr'].high, negative_energies['r'].high): direction.zero if is_direction else velocity.fast,  # rule no. 13
                     (negative_energies['l'].high, negative_energies['fl'].low, negative_energies['f'].low, negative_energies['fr'].high, negative_energies['r'].low): direction.zero if is_direction else velocity.fast,  # rule no. 14
                     (negative_energies['l'].high, negative_energies['fl'].low, negative_energies['f'].low, negative_energies['fr'].low, negative_energies['r'].high): direction.zero if is_direction else velocity.fast,  # rule no. 15
                     (negative_energies['l'].high, negative_energies['fl'].low, negative_energies['f'].low, negative_energies['fr'].low, negative_energies['r'].low): direction.zero if is_direction else velocity.fast,  # rule no. 16
                     (negative_energies['l'].low, negative_energies['fl'].high, negative_energies['f'].high, negative_energies['fr'].high, negative_energies['r'].high): direction.large_neg if is_direction else velocity.slow,  # rule no. 17
                     (negative_energies['l'].low, negative_energies['fl'].high, negative_energies['f'].high, negative_energies['fr'].high, negative_energies['r'].low): direction.large_pos if is_direction else velocity.slow,  # rule no. 18
                     (negative_energies['l'].low, negative_energies['fl'].high, negative_energies['f'].high, negative_energies['fr'].low, negative_energies['r'].high): direction.small_pos if is_direction else velocity.slow,  # rule no. 19
                     (negative_energies['l'].low, negative_energies['fl'].high, negative_energies['f'].high, negative_energies['fr'].low, negative_energies['r'].low): direction.large_pos if is_direction else velocity.slow,  # rule no. 20
                     (negative_energies['l'].low, negative_energies['fl'].high, negative_energies['f'].low, negative_energies['fr'].high, negative_energies['r'].high): direction.zero if is_direction else velocity.fast,  # rule no. 21
                     (negative_energies['l'].low, negative_energies['fl'].high, negative_energies['f'].low, negative_energies['fr'].high, negative_energies['r'].low): direction.zero if is_direction else velocity.fast,  # rule no. 22
                     (negative_energies['l'].low, negative_energies['fl'].high, negative_energies['f'].low, negative_energies['fr'].low, negative_energies['r'].high): direction.zero if is_direction else velocity.fast,  # rule no. 23
                     (negative_energies['l'].low, negative_energies['fl'].high, negative_energies['f'].low, negative_energies['fr'].low, negative_energies['r'].low): direction.zero if is_direction else velocity.fast,  # rule no. 24
                     (negative_energies['l'].low, negative_energies['fl'].low, negative_energies['f'].high, negative_energies['fr'].high, negative_energies['r'].high): direction.large_neg if is_direction else velocity.slow,  # rule no. 25
                     (negative_energies['l'].low, negative_energies['fl'].low, negative_energies['f'].high, negative_energies['fr'].high, negative_energies['r'].low): direction.large_neg if is_direction else velocity.slow,  # rule no. 26
                     (negative_energies['l'].low, negative_energies['fl'].low, negative_energies['f'].high, negative_energies['fr'].low, negative_energies['r'].high): direction.large_neg if is_direction else velocity.slow,  # rule no. 27
                     (negative_energies['l'].low, negative_energies['fl'].low, negative_energies['f'].high, negative_energies['fr'].low, negative_energies['r'].low): direction.large_pos if is_direction else velocity.slow,  # rule no. 28
                     (negative_energies['l'].low, negative_energies['fl'].low, negative_energies['f'].low, negative_energies['fr'].high, negative_energies['r'].high): direction.zero if is_direction else velocity.fast,  # rule no. 29
                     (negative_energies['l'].low, negative_energies['fl'].low, negative_energies['f'].low, negative_energies['fr'].high, negative_energies['r'].low): direction.zero if is_direction else velocity.fast,  # rule no. 30
                     (negative_energies['l'].low, negative_energies['fl'].low, negative_energies['f'].low, negative_energies['fr'].low, negative_energies['r'].high): direction.zero if is_direction else velocity.fast,  # rule no. 31
                     (negative_energies['l'].low, negative_energies['fl'].low, negative_energies['f'].low, negative_energies['fr'].low, negative_energies['r'].low): direction.zero if is_direction else velocity.fast,  # rule no. 32
                     })

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

    def __calculate_obstacle_impact(self, obstacles):
        """ Calculates obstacle impact sum for all objects in one sector. A part of negative energy calculation. 
        :param obstacles: a list of obstacles represented by tuples (occupied_angle, distance) of obstacle
        :return: obstacle impact for one sector within the field of view of a pedestrian
        """

        occupied_angle = self.domains.occupied_angle
        distance = self.domains.goal_distance
        oi = self.domains.obstacle_impact
        
        rules_oi = self.__create_obstacle_impact_rules(oi, occupied_angle, distance)

        obstacleImpact = 0
        for (a, d) in obstacles:
            values = {occupied_angle: a, distance: d}
            obstacleImpact += rules_oi(values)

        return obstacleImpact

    @staticmethod
    def __create_obstacle_impact_rules(oi, occupied_angle, distance):
        """ Obstacle impact rules: an obstacle which has nearer distance and larger occupied angle brings larger OI.
        :param oi: obstacle impact
        :param occupied_angle: angle that the obstacle occupies in the field of vision
        :param distance: distance to obstacle
        :return: constructed rules
        """

        return Rule({(occupied_angle.large, distance.near): oi.large,
                     (occupied_angle.large, distance.far): oi.middle,
                     (occupied_angle.small, distance.near): oi.middle,
                     (occupied_angle.small, distance.far): oi.small,
                     })

    def __calculate_collision_risk(self, pedestrians):
        """ Calculates collision risk sum for all pedestrians in sector.
        :param pedestrians: a list of pedestrians represented by tuples (distance, velocity, angle)
        :return: collision risk for one of the five angled areas within the field of view of a pedestrian
        """

        distance = self.domains.goal_distance
        velocity = self.domains.velocity
        angle = self.domains.goal_angle
        cr = self.domains.collision_risk

        rules_cr = self.__create_collision_risk_rules(cr, distance, velocity, angle)

        collisionRisk = 0
        for (d, v, a) in pedestrians:
            values = {distance: d, velocity: v, angle: a}
            collisionRisk += rules_cr(values)

        return collisionRisk

    @staticmethod
    def __create_collision_risk_rules(cr, distance, velocity, angle):
        """ Collision risk rules: lower CR if a opposite pedestrian deviated from the decision maker, had a far relative distance or moved with a low speed. 
        :param cr: collision risk
        :param distance: distance to pedestrian
        :param velocity: pedestrian movement speed
        :param angle: angle between pedestrian and decision maker
        
        :return: constructed rules
        """

        return Rule({(distance.near, velocity.stop, angle.large_neg): cr.low,
                     (distance.near, velocity.stop, angle.small_neg): cr.low,
                     (distance.near, velocity.stop, angle.zero): cr.low,
                     (distance.near, velocity.stop, angle.small_pos): cr.low,
                     (distance.near, velocity.stop, angle.large_pos): cr.low,
                     (distance.near, velocity.slow, angle.large_neg): cr.low,
                     (distance.near, velocity.slow, angle.small_neg): cr.high,
                     (distance.near, velocity.slow, angle.zero): cr.high,
                     (distance.near, velocity.slow, angle.small_pos): cr.high,
                     (distance.near, velocity.slow, angle.large_pos): cr.low,
                     (distance.near, velocity.fast, angle.large_neg): cr.low,
                     (distance.near, velocity.fast, angle.small_neg): cr.high,
                     (distance.near, velocity.fast, angle.zero): cr.high,
                     (distance.near, velocity.fast, angle.small_pos): cr.high,
                     (distance.near, velocity.fast, angle.large_pos): cr.low,
                     (distance.far, velocity.stop, angle.large_neg): cr.low,
                     (distance.far, velocity.stop, angle.small_neg): cr.low,
                     (distance.far, velocity.stop, angle.zero): cr.low,
                     (distance.far, velocity.stop, angle.small_pos): cr.low,
                     (distance.far, velocity.stop, angle.large_pos): cr.low,
                     (distance.far, velocity.slow, angle.large_neg): cr.low,
                     (distance.far, velocity.slow, angle.small_neg): cr.low,
                     (distance.far, velocity.slow, angle.zero): cr.low,
                     (distance.far, velocity.slow, angle.small_pos): cr.low,
                     (distance.far, velocity.slow, angle.large_pos): cr.low,
                     (distance.far, velocity.fast, angle.large_neg): cr.low,
                     (distance.far, velocity.fast, angle.small_neg): cr.high,
                     (distance.far, velocity.fast, angle.zero): cr.high,
                     (distance.far, velocity.fast, angle.small_pos): cr.high,
                     (distance.far, velocity.fast, angle.large_pos): cr.low,
                     })

    def __calculate_negative_energies(self, obstacles, pedestrians) -> Dict[str, float]:
        """ Calculates negative energies for all sectors.
        :param obstacles: a dictionary of obstacles represented by a list of tuples (occupied_angle, distance) for each field of vision
        :params pedestrians: a dictionary of pedestrians represented by a list of tuples (distance, velocity, angle) for each field of vision
        :return: normalized negative energies within the five angled areas within the field of view of a pedestrian
        """

        negativeEnergyPerSector = {'l': 0.0, 'fl': 0.0, 'f': 0.0, 'fr': 0.0, 'r': 0.0}
        sectors = ['l', 'fl', 'f', 'fr', 'r']

        for s in sectors:
            obstacleImpactPerSector = self.__calculate_obstacle_impact(obstacles[s])
            collisionRiskPerSector = self.__calculate_collision_risk(pedestrians[s])
            negativeEnergyPerSector[s] = Environment.k_w * obstacleImpactPerSector + (1 - Environment.k_w) * collisionRiskPerSector

        #Normalize negative energies
        minimum = min(negativeEnergyPerSector.values())
        maximum = max(negativeEnergyPerSector.values())
        for i in negativeEnergyPerSector.keys():
            negativeEnergyPerSector[i] = (maximum - negativeEnergyPerSector[i]) / (maximum - minimum)

        return negativeEnergyPerSector
    
    def __collision_detection(self, obstacle: 'Obstacle'):
        for ov1, ov2 in zip(obstacle.get_vertices()[:-1], obstacle.get_vertices()[1:]):
            for i, ray in enumerate(self.ray_coords):
                intersection = Utils.find_intersection(ov1, ov2, self.coordinates, ray)
                if np.all(intersection):
                    self.ray_intersections[i].append(intersection)
                    if i > 0:
                        self.ray_intersections[i-1].append(intersection)

            for i, ray1, ray2 in zip(range(5), self.ray_coords[:-1], self.ray_coords[1:]):
                intersection = Utils.find_intersection(ov1, ov2, ray1, ray2)
                if np.all(intersection):
                    self.ray_intersections[i].append(intersection)

                if Utils.is_inside_triangle(ov1,self.coordinates,ray1,ray2):
                    self.ray_intersections[i].append(ov1)
                if Utils.is_inside_triangle(ov2,self.coordinates,ray1,ray2):
                    self.ray_intersections[i].append(ov2)
        

    def __update_position(self, move, angle):
        self.coordinates += move
        self.angle += angle
        self.angle %= 360
        self.angle_rad = np.radians(self.angle)
        
        self.ray_coords = np.array([(np.cos(angle + self.angle_rad), np.sin(angle + self.angle_rad)) for angle in Environment.fov_ray_angles_rad])
        self.ray_coords = self.coordinates + Environment.dmax * self.ray_coords
    
    def __update_collisions(self):
        self.ray_intersections = [[] for _ in range(6)]
        for obstacle in self.simulation.obstacles:
            self.__collision_detection(obstacle)
            
        self.zone_obstacle_dist = {}
        for zone_name, ray in zip(Environment.zone_names, self.ray_intersections):
            if len(ray) != 0:
                self.zone_obstacle_dist[zone_name] = np.min(np.linalg.norm(ray - self.coordinates,axis=1))
            else:
                self.zone_obstacle_dist[zone_name] = Environment.dmax + 1

    def update_from_keyboard(self, pressed_keys):
        move = 50 / self.simulation.tick_rate * np.array([np.cos(self.angle_rad), np.sin(self.angle_rad)])
        self.__update_collisions()
        if pressed_keys[pygame.K_UP] or pressed_keys[pygame.K_w]:
            self.__update_position(move, 0)
        if pressed_keys[pygame.K_DOWN] or pressed_keys[pygame.K_s]:
            self.__update_position(-move, 0)
        if pressed_keys[pygame.K_LEFT] or pressed_keys[pygame.K_a]:
            self.__update_position(0, -5)
        if pressed_keys[pygame.K_RIGHT] or pressed_keys[pygame.K_d]:
            self.__update_position(0, 5)
    
    def __get_update_params(self):
        (angle_1, velocity_1) = self.__local_obstacle_avoiding_behavior(self.zone_obstacle_dist)  # TODO: replace hardcoded distance with the nearest object distance from object detection algorithm
        if abs(angle_1) < 1:
            angle_1 = 0
        
        #(angle_2, velocity_2) = self.__regional_path_searching_behavior(self.__calculate_negative_energies({'l': [[15, 3.41]], 'fl': [[1, 2.3], [1, 3.3]], 'f': [[5, 3.1]], 'fr': [[4, 5.0]], 'r': [[2, 4.8]]}, {'l': [[3.41, 1, 1]], 'fl': [[2.3, 1, 1]], 'f': [[3.1, 1, 1]], 'fr': [[5.0, 1, 1]], 'r': [[4.8, 1, 1]]})) # TODO: replace hardcoded data for obstacles and pedestrians in field of vision from object and pedestrian detection algorithms
        
        goal_angle, goal_dist = angle_between(self, self.goal), distance_between(self.coordinates, self.goal.coordinates)
        (angle_3, velocity_3) = self.__global_goal_seeking_behavior(goal_angle, goal_dist)
        
        #movement_speed, turning_angle = self.__integrate_multiple_behaviors([angle_1, angle_2, angle_3], [velocity_1, velocity_2, velocity_3])
        #print(f'Turning angle: {turning_angle}, movement speed: {movement_speed}')  # TODO: connect output values with visualization
        
        movement_speed, turning_angle = self.__integrate_multiple_behaviors([angle_1, angle_3], [velocity_1, velocity_3])
        # if self.coordinates[1] > self.simulation.goal.coordinates[1]:
        #     movement_speed = -movement_speed
        
        #movement_speed*=0.05
        return movement_speed, turning_angle
    
    def __is_in_goal(self):
        return np.linalg.norm(self.coordinates - self.goal.coordinates) < Environment.goal_tolerance
    
    def update(self):
        self.__update_collisions()
        movement_speed, turning_angle = self.__get_update_params()

        self.draw()
        
        move = movement_speed / self.simulation.tick_rate * np.array([np.cos(self.angle_rad), np.sin(self.angle_rad)])
        self.__update_position(move,turning_angle)
        
        if self.__is_in_goal():
            self.simulation.remove_pedestrian(self)

    def draw(self):
        coords_scaled = self.coordinates * self.simulation.scale
        ray_coords_scaled = self.ray_coords * self.simulation.scale
        
        for ray in ray_coords_scaled:
            pygame.draw.line(self.simulation.screen, Environment.color_black, coords_scaled, ray, 1)

        for ray1, ray2 in zip(ray_coords_scaled[:-1], ray_coords_scaled[1:]):
            pygame.draw.line(self.simulation.screen, Environment.color_black, ray1, ray2, 1)
        
        
        for fov_segment_intersections in self.ray_intersections:
            fov_segment_intersections_scaled = np.array(fov_segment_intersections) * self.simulation.scale
            for ray_intersection in fov_segment_intersections_scaled:
                pygame.draw.circle(self.simulation.screen, Environment.color_red, ray_intersection, 3)
                
        if self.manual:
            movement_speed, turning_angle = self.__get_update_params()
            goal_angle, goal_dist = angle_between(self, self.goal), distance_between(self.coordinates, self.goal.coordinates)
            
            y_offset = 0
            x_offset = 0
            texts = []
            texts.append((Environment.font.render(f'Angle: {self.angle}',True, Environment.color_black),(x_offset,y_offset)))
            y_offset+=16
            texts.append((Environment.font.render(f'Goal: {goal_angle:.0f} {goal_dist:.0f}',True, Environment.color_black),(x_offset,y_offset)))
            y_offset+=16
            texts.append((Environment.font.render(f'Obstacle dist: {",".join([f"{d:.2f}" for d in self.zone_obstacle_dist.values()])}',True, Environment.color_black),(x_offset,y_offset)))
            y_offset+=16
            texts.append((Environment.font.render(f'Move: {turning_angle:.0f} {movement_speed:.2f}',True, Environment.color_black),(x_offset,y_offset)))
            self.simulation.screen.blits(texts)

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

    def get_coordinates(self):
        return self.coordinates

    def get_angle(self):
        return self.angle
