from typing import Tuple, Dict, List
from fuzzylogic.classes import Rule
from algebra_functions import distance_between, intersects, unit_vector_between
from environment import Environment
from simulation import Simulation
import numpy as np


class Assailant:
    def __init__(self, simulation: Simulation, position: Tuple[float, float], angle: float, velocity: 0.0):
        if angle < 0.0 or angle > 359.0:
            raise IOError(
                "Valid direction angles for a pedestrian are within 0 and 359 degrees.")  # 360 would be equal to 0 and thus result in issues with the fuzzy logic rule application, i.e.

        # Data of the simulation
        self.simulation = simulation
        self.domains = self.simulation.get_domains()
        self.targets = self.simulation.get_pedestrians()

        # Assailant's properties
        self.coordinates = position  # current position P_n
        self.angle = angle  # direction X_in
        self.velocity = velocity  # movement speed V_n

    def __filter_targets__(self): #only return targets that aren't behind walls
        filtered_targets = []
        for pedestrian in self.targets:
            valid = True
            for obstacle in self.simulation.get_obstacles():
                for i in range(len(obstacle.vertices)-1):
                    if intersects((self.coordinates, pedestrian.get_coordinates()), (obstacle.vertices[i], obstacle.vertices[i+1])):
                        valid = False
                        break

                if not valid:
                    break

            if valid:
                filtered_targets.append(pedestrian)

        return filtered_targets


    def target_selection_distance(self, key=min):  # -> Tuple[float, float]:  # returns the coordinates of the assailants's target based on distance from assailant.
        """ Selects a target using distance from assailant
        :param key: a function that accepts *n* float values and returns one of them
        :return: tuple of current direction/angle and velocity/movement speed based on the corresponding rules
        """

        #TODO: change return value to angle.
        return key((distance_between(pedestrian.get_coordinates(), self.coordinates), pedestrian.get_coordinates()) for pedestrian in self.__filter_targets__())

    def target_selection_peripherality(self, key=max): #-> Tuple[float, float]:
        """ Selects a target using peripherality of group
        :param key: a function that accepts *n* float values and returns one of them
        :return: tuple of current direction/angle and velocity/movement speed based on the corresponding rules
        """
        targets = self.__filter_targets__()
        scores = []
        for pedestrian in targets:
            vector_sum = [sum(element) for element in zip(*[unit_vector_between(pedestrian.get_coordinates(), self.coordinates) for p in targets if p.get_coordinates() != pedestrian.get_coordinates()])]
            vector_sum = [e/max((len(targets)-1), 1) for e in vector_sum]
            peripherality = np.sqrt(vector_sum[0]**2 + vector_sum[1]**2)
            scores.append((peripherality, pedestrian.get_coordinates()))

        return scores#key(scores)


    def __regional_path_searching_behavior(self):  # TODO: Assaliant still needs to avoid obstacles
        pass

    def __local_obstacle_avoiding_behavior(self, nearest_object_distances: Dict[str, float]) -> Tuple[
        float, float]:  # TODO: Same function as in pedestrians?
        pass
