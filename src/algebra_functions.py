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

def angle_between(params): #TODO: Move angle calculating function from pedestrians here
    pass