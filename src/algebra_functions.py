from typing import Tuple

import numpy as np

def counterclockwise(point_1, point_2, point_3):
    """
    :param point_1: a tuple representing cartesian coordinates
    :param point_2: a tuple representing cartesian coordinates
    :param point_3: a tuple representing cartesian coordinates
    :return: whether the triangle p1p2p3 is rotated counter-clockwise
    """
    return (point_3[1] - point_1[1]) * (point_2[0] - point_1[0]) > (point_2[1] - point_1[1]) * (point_3[0] - point_1[0])

def intersects(segment_1, segment_2):
    """
    :param segment_1: a tuple representing two points in a cartesian coordinate system
    :param segment_2: a tuple representing two points in a cartesian coordinate system
    :return: whether the two segments intersect
    """
    return counterclockwise(segment_1[0], segment_2[0], segment_2[1]) != counterclockwise(segment_1[1], segment_2[0],segment_2[1]) and counterclockwise(segment_1[0], segment_1[1], segment_2[0]) != counterclockwise(segment_1[0], segment_1[1], segment_2[1])

def distance_between(point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
    """
    :param point1: a tuple representing cartesian coordinates
    :param point2: a tuple representing cartesian coordinates
    :return: distance between point1 and point2
    """
    return np.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

def  unit_vector_between(point_from: Tuple[float,float], point_to: Tuple[float,float]) -> Tuple[float,float]:
    directional_vector = (point_to[0]-point_from[0],point_to[1]-point_from[1])
    vector_length = np.sqrt(directional_vector[0]**2 + directional_vector[1]**2)
    return directional_vector/vector_length

def angle_between(actor_from, actor_to):
    """ Calculates angle between first and second actor. Positive angles denote that the goal is on the right side of the first actor. Angles are calculated in degrees.
    :return: angle(pedestrian, goal) in degrees
    """
    p = np.asarray(actor_from.coordinates)  # first actor
    x = np.asarray([actor_from.coordinates[0] + 5, actor_from.coordinates[1]])  # x-axis
    g = np.asarray(actor_to.coordinates)  # second actor

    px = x - p
    pg = g - p

    alpha = np.degrees(np.arccos(np.dot(px, pg) / (np.linalg.norm(px) * np.linalg.norm(pg))))  # angle between actor_to, actor_from and x-axis (our angle origin)

    if actor_to.coordinates[1] < actor_from.coordinates[1]:
        alpha = 360 - alpha

    goal_angle = actor_from.angle - alpha

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