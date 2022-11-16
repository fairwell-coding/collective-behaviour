import matplotlib.pyplot as plt
from fuzzylogic.classes import Domain
from fuzzylogic.functions import trapezoid, rectangular, R, S


class FuzzyDomains:

    def __init__(self):

        # self.__create_goal_angle_domain()
        self.__create_goal_distance_domain()
        # TODO: Define and call all other needed domains here as private methods inside constructor

    def __create_goal_distance_domain(self) -> None:
        self.goal_distance = Domain("goal distance", 0, 5, res=0.1)

        self.goal_distance.near_rect = rectangular(0, 1, c_m=1.0)
        self.goal_distance.near_lin = S(1, 2)
        self.goal_distance.near = self.goal_distance.near_rect + self.goal_distance.near_lin
        self.goal_distance.near.plot()
        plt.show()

        self.goal_distance.far_lin = R(1, 2)
        self.goal_distance.far_rect = rectangular(2, 5, c_m=1.0)
        self.goal_distance.far = self.goal_distance.far_lin + self.goal_distance.far_rect
        self.goal_distance.far.plot()
        plt.show()

    def __create_goal_angle_domain(self) -> None:
        self.goal_angle = Domain("goal angle", 0, 360)

        self.goal_angle.large_neg_rect = rectangular(0, 30, c_m=1.0)
        self.goal_angle.large_neg_linear = S(30, 60)
        self.goal_angle.large_neg = self.goal_angle.large_neg_rect + self.goal_angle.large_neg_linear
        self.goal_angle.large_neg.plot()
        plt.show()

        self.goal_angle.small_neg = trapezoid(30, 60, 120, 150, c_m=1.0)
        self.goal_angle.zero = trapezoid(120, 150, 210, 240, c_m=1.0)
        self.goal_angle.small_pos = trapezoid(210, 240, 300, 330, c_m=1.0)
        self.goal_angle.small_neg.plot()
        plt.show()
        self.goal_angle.zero.plot()
        plt.show()
        self.goal_angle.small_pos.plot()
        plt.show()

        self.goal_angle.large_pos_lin = R(300, 330)
        self.goal_angle.large_pos_rect = rectangular(330, 359, c_m=1.0)
        self.goal_angle.large_pos = self.goal_angle.large_pos_lin + self.goal_angle.large_pos_rect
        self.goal_angle.large_pos.plot()
        plt.show()

    def get_goal_angle_domain(self) -> Domain:
        return self.goal_angle

    def get_goal_distance_domain(self) -> Domain:
        return self.goal_distance
