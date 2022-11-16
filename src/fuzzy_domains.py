import matplotlib.pyplot as plt
from fuzzylogic.classes import Domain
from fuzzylogic.functions import trapezoid, rectangular, linear, bounded_linear, R, S


class FuzzyDomains:

    def __init__(self, res=0.1):
        self.res = res

        self.__create_goal_angle_domain()
        # TODO: Define and call all other needed domains here as private methods inside constructor

    def __create_goal_angle_domain(self) -> None:
        self.goal_angle = Domain("goal angle", 0, 360, res=self.res)

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

    def get_goal_distance_domain(self):
        pass
