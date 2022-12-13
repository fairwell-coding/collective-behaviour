import numpy as np

class Utils:
    
    @staticmethod
    def find_intersection(s1, d1, s2, d2):
        s1, d1, s2, d2 = np.vstack([s1, d1, s2, d2])
        direction1 = d1-s1
        direction2 = d2-s2

        direction_cross = np.cross(direction1, direction2)
        origin_cross1 = np.cross(s2-s1, direction2)
        origin_cross2 = np.cross(s2-s1, direction1)
        if direction_cross != 0:
            n1 = origin_cross1 / direction_cross
            n2 = origin_cross2 / direction_cross
            if 0 <= n1 <= 1 and 0 <= n2 <= 1:
                p = s1 + direction1 * n1
                return p
            else:
                return None
        else:
            return None

    @staticmethod
    def sign(p1, p2, p3):
        return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1]) < 0

    @staticmethod
    def is_inside_triangle(p, p1, p2, p3):
        sings = [Utils.sign(p,p1,p2),Utils.sign(p,p2,p3),Utils.sign(p,p3,p1)]
        return not np.any(sings) or np.all(sings)