import matplotlib.pyplot as plt
from fuzzylogic.classes import Domain
from fuzzylogic.functions import trapezoid, rectangular, R, S
from environment import Environment


class FuzzyDomains:
    """ Represents all domains and its corresponding fuzzy sets. Upon object instantiation all required membership functions are defined accordingly.
    """

    def __init__(self, plot_membership_functions=False):
        self.plot_membership_functions = plot_membership_functions  # used for debugging

        self.__create_direction_domain()  # fig. 5a (vision of a pedestrian)
        self.__create_goal_angle_domain()  # similar defined to fig. 5a (not same angle range)
        self.__create_distance_domain()  # fig. 5c
        self.__create_velocity_domain()  # fig. 5b
        self.__create_goal_distance_domain()  # fig. 5e

    def __create_distance_domain(self):
        self.distance = Domain("distance", 0, Environment.dmax, res=.1)

        self.distance.near_rect = rectangular(0, 5/19 * Environment.dmax, c_m=1.0)
        self.distance.near_lin = S(5/19 * Environment.dmax, 8/19 * Environment.dmax)
        self.distance.near = self.distance.near_rect + self.distance.near_lin
        if self.plot_membership_functions:
            self.distance.near.plot()
            plt.show()

        self.distance.far_lin = R(5/19 * Environment.dmax, 8/19 * Environment.dmax)
        self.distance.far_rect = rectangular(8/19 * Environment.dmax, Environment.dmax, c_m=1.0)
        self.distance.far = self.distance.far_lin + self.distance.far_rect
        if self.plot_membership_functions:
            self.distance.far.plot()
            plt.show()

    def __create_direction_domain(self):
        self.direction = Domain("direction", 0, Environment.fov, res=1.0)

        self.direction.large_neg_rect = rectangular(0, 1/12 * Environment.fov, c_m=1.0)
        self.direction.large_neg_linear = S(1/12 * Environment.fov, 1/6 * Environment.fov)
        self.direction.large_neg = self.direction.large_neg_rect + self.direction.large_neg_linear
        if self.plot_membership_functions:
            self.direction.large_neg.plot()
            plt.show()

        self.direction.small_neg = trapezoid(1/12 * Environment.fov, 1/6 * Environment.fov, 1/3 * Environment.fov, 5/12 * Environment.fov, c_m=1.0)
        self.direction.zero = trapezoid(1/3 * Environment.fov, 5/12 * Environment.fov, 7/12 * Environment.fov, 2/3 * Environment.fov, c_m=1.0)
        self.direction.small_pos = trapezoid(7/12 * Environment.fov, 2/3 * Environment.fov, 5/6 * Environment.fov, 11/12 * Environment.fov, c_m=1.0)
        if self.plot_membership_functions:
            self.direction.small_neg.plot()
            plt.show()
            self.direction.zero.plot()
            plt.show()
            self.direction.small_pos.plot()
            plt.show()

        self.direction.large_pos_lin = R(5/6 * Environment.fov, 11/12 * Environment.fov)
        self.direction.large_pos_rect = rectangular(11/12 * Environment.fov, Environment.fov, c_m=1.0)
        self.direction.large_pos = self.direction.large_pos_lin + self.direction.large_pos_rect
        if self.plot_membership_functions:
            self.direction.large_pos.plot()
            plt.show()

    def __create_velocity_domain(self):
        self.velocity = Domain("velocity", 0, Environment.vmax, res=.1)

        self.velocity.stop_rect = rectangular(0, 1 / 6 * Environment.vmax)
        self.velocity.stop_lin = S(1/6 * Environment.vmax, 1/4 * Environment.vmax)
        self.velocity.stop = self.velocity.stop_rect + self.velocity.stop_lin
        if self.plot_membership_functions:
            self.velocity.stop.plot()
            plt.show()

        self.velocity.slow = trapezoid(1/6 * Environment.vmax, 1/4 * Environment.vmax, 3/4 * Environment.vmax, 5/6 * Environment.vmax)
        if self.plot_membership_functions:
            self.velocity.slow.plot()
            plt.show()

        self.velocity.fast_lin = R(3/4 * Environment.vmax, 5/6 * Environment.vmax)
        self.velocity.fast_rect = rectangular(5/6 * Environment.vmax, Environment.vmax)
        self.velocity.fast = self.velocity.fast_lin + self.velocity.fast_rect
        if self.plot_membership_functions:
            self.velocity.fast.plot()
            plt.show()

    def __create_goal_distance_domain(self) -> None:
        self.goal_distance = Domain("goal distance", 0, 5, res=0.1)

        self.goal_distance.near_rect = rectangular(0, 1, c_m=1.0)
        self.goal_distance.near_lin = S(1, 2)
        self.goal_distance.near = self.goal_distance.near_rect + self.goal_distance.near_lin
        if self.plot_membership_functions:
            self.goal_distance.near.plot()
            plt.show()

        self.goal_distance.far_lin = R(1, 2)
        self.goal_distance.far_rect = rectangular(2, 5, c_m=1.0)
        self.goal_distance.far = self.goal_distance.far_lin + self.goal_distance.far_rect
        if self.plot_membership_functions:
            self.goal_distance.far.plot()
            plt.show()

    def __create_goal_angle_domain(self) -> None:
        self.goal_angle = Domain("goal angle", 0, 360)

        self.goal_angle.large_neg_rect = rectangular(0, 30, c_m=1.0)
        self.goal_angle.large_neg_linear = S(30, 60)
        self.goal_angle.large_neg = self.goal_angle.large_neg_rect + self.goal_angle.large_neg_linear
        if self.plot_membership_functions:
            self.goal_angle.large_neg.plot()
            plt.show()

        self.goal_angle.small_neg = trapezoid(30, 60, 120, 150, c_m=1.0)
        self.goal_angle.zero = trapezoid(120, 150, 210, 240, c_m=1.0)
        self.goal_angle.small_pos = trapezoid(210, 240, 300, 330, c_m=1.0)
        if self.plot_membership_functions:
            self.goal_angle.small_neg.plot()
            plt.show()
            self.goal_angle.zero.plot()
            plt.show()
            self.goal_angle.small_pos.plot()
            plt.show()

        self.goal_angle.large_pos_lin = R(300, 330)
        self.goal_angle.large_pos_rect = rectangular(330, 359, c_m=1.0)
        self.goal_angle.large_pos = self.goal_angle.large_pos_lin + self.goal_angle.large_pos_rect
        if self.plot_membership_functions:
            self.goal_angle.large_pos.plot()
            plt.show()

    def get_goal_angle_domain(self) -> Domain:
        return self.goal_angle

    def get_goal_distance_domain(self) -> Domain:
        return self.goal_distance

    def get_velocity_domain(self) -> Domain:
        return self.velocity

    def get_direction_domain(self) -> Domain:
        return self.direction

    def get_distance_domain(self) -> Domain:
        return self.distance
