from typing import Dict

import matplotlib.pyplot as plt
from fuzzylogic.classes import Domain
from fuzzylogic.functions import trapezoid, rectangular, R, S

from environment import Environment


class FuzzyDomains:
    """ Represents all domains and its corresponding fuzzy sets. Upon object instantiation all required membership functions are defined accordingly.
    """

    def __init__(self, plot_membership_functions=False):
        self.plot_membership_functions = plot_membership_functions  # used for debugging

        self.__create_fov_domain()  # fig. 5a (direction/vision/field of view of a pedestrian)
        self.__create_goal_angle_domain()  # similar defined to fig. 5a (not same angle range)
        self.__create_distance_domains()  # fig. 5c
        self.__create_velocity_domain()  # fig. 5b
        self.__create_negative_energy_domains()  # fig. 5d        
        self.__create_goal_distance_domain()  # fig. 5e

    def __create_distance_domains(self):
        """ Creates all five distance domains within the field of view of each pedestrian.
        """

        self.distances = {'l': self.__create_distance_domain(),  # left
                          'fl': self.__create_distance_domain(),  # front left
                          'f': self.__create_distance_domain(),  # front
                          'fr': self.__create_distance_domain(),  # front right
                          'r': self.__create_distance_domain()  # right
                          }

    def __create_distance_domain(self) -> Domain:
        """ Returns local domain object since this one needs to be copied
        :return: prototype distance domain object
        """

        distance = Domain("distance", 0, Environment.dmax, res=.1)

        distance.near_rect = rectangular(0, 5 / 19 * Environment.dmax, c_m=1.0)
        distance.near_lin = S(5 / 19 * Environment.dmax, 8 / 19 * Environment.dmax)
        distance.near = distance.near_rect + distance.near_lin
        if self.plot_membership_functions:
            distance.near.plot()
            plt.show()

        distance.far_lin = R(5 / 19 * Environment.dmax, 8 / 19 * Environment.dmax)
        distance.far_rect = rectangular(8 / 19 * Environment.dmax, Environment.dmax, c_m=1.0)
        distance.far = distance.far_lin + distance.far_rect
        if self.plot_membership_functions:
            distance.far.plot()
            plt.show()

        return distance

    def __create_fov_domain(self):
        self.fov = Domain("fov", -1 / 2 * Environment.fov, 1 / 2 * Environment.fov, res=1.0)

        self.fov.large_neg_rect = rectangular(-1 / 2 * Environment.fov, - 5 / 12 * Environment.fov, c_m=1.0)
        self.fov.large_neg_linear = S(-5 / 12 * Environment.fov, -1 / 3 * Environment.fov)
        self.fov.large_neg = self.fov.large_neg_rect + self.fov.large_neg_linear
        if self.plot_membership_functions:
            self.fov.large_neg.plot()
            plt.show()

        self.fov.small_neg = trapezoid(-5 / 12 * Environment.fov, -1 / 3 * Environment.fov, - 1 / 6 * Environment.fov, -1 / 12 * Environment.fov, c_m=1.0)
        self.fov.zero = trapezoid(-1 / 6 * Environment.fov, -1 / 12 * Environment.fov, 1 / 12 * Environment.fov, 1 / 6 * Environment.fov, c_m=1.0)
        self.fov.small_pos = trapezoid(1 / 12 * Environment.fov, 1 / 6 * Environment.fov, 1 / 3 * Environment.fov, 5 / 12 * Environment.fov, c_m=1.0)
        if self.plot_membership_functions:
            self.fov.small_neg.plot()
            plt.show()
            self.fov.zero.plot()
            plt.show()
            self.fov.small_pos.plot()
            plt.show()

        self.fov.large_pos_lin = R(1 / 3 * Environment.fov, 5 / 12 * Environment.fov)
        self.fov.large_pos_rect = rectangular(5 / 12 * Environment.fov, 1 / 2 * Environment.fov, c_m=1.0)
        self.fov.large_pos = self.fov.large_pos_lin + self.fov.large_pos_rect
        if self.plot_membership_functions:
            self.fov.large_pos.plot()
            plt.show()

    def __create_velocity_domain(self):
        self.velocity = Domain("velocity", 0, Environment.vmax, res=.1)

        self.velocity.stop_rect = rectangular(0, 1 / 6 * Environment.vmax)
        self.velocity.stop_lin = S(1 / 6 * Environment.vmax, 1 / 4 * Environment.vmax)
        self.velocity.stop = self.velocity.stop_rect + self.velocity.stop_lin
        if self.plot_membership_functions:
            self.velocity.stop.plot()
            plt.show()

        self.velocity.slow = trapezoid(1 / 6 * Environment.vmax, 1 / 4 * Environment.vmax, 3 / 4 * Environment.vmax, 5 / 6 * Environment.vmax)
        if self.plot_membership_functions:
            self.velocity.slow.plot()
            plt.show()

        self.velocity.fast_lin = R(3 / 4 * Environment.vmax, 5 / 6 * Environment.vmax)
        self.velocity.fast_rect = rectangular(5 / 6 * Environment.vmax, Environment.vmax)
        self.velocity.fast = self.velocity.fast_lin + self.velocity.fast_rect
        if self.plot_membership_functions:
            self.velocity.fast.plot()
            plt.show()

    def __create_goal_distance_domain(self) -> None:
        self.goal_distance = Domain("goal distance", 0, Environment.gmax, res=0.1)

        self.goal_distance.near_rect = rectangular(0, 5, c_m=1.0)
        self.goal_distance.near_lin = S(5, 7.5)
        self.goal_distance.near = self.goal_distance.near_rect + self.goal_distance.near_lin
        if self.plot_membership_functions:
            self.goal_distance.near.plot()
            plt.show()

        self.goal_distance.far_lin = R(5, 7.5)
        self.goal_distance.far_rect = rectangular(7.5, Environment.gmax, c_m=1.0)
        self.goal_distance.far = self.goal_distance.far_lin + self.goal_distance.far_rect
        if self.plot_membership_functions:
            self.goal_distance.far.plot()
            plt.show()

    def __create_goal_angle_domain(self) -> None:
        self.goal_angle = Domain("goal angle", -1 / 2 * Environment.g_angle_max, 1 / 2 * Environment.g_angle_max)

        self.goal_angle.large_neg_rect = rectangular(-1 / 2 * Environment.g_angle_max + 1, -5 / 12 * Environment.g_angle_max, c_m=1.0)
        self.goal_angle.large_neg_linear = S(-5 / 12 * Environment.g_angle_max, -1 / 3 * Environment.g_angle_max)
        self.goal_angle.large_neg = self.goal_angle.large_neg_rect + self.goal_angle.large_neg_linear
        if self.plot_membership_functions:
            self.goal_angle.large_neg.plot()
            plt.show()

        self.goal_angle.small_neg = trapezoid(-5 / 12 * Environment.g_angle_max, -1 / 3 * Environment.g_angle_max, -1 / 6 * Environment.g_angle_max, -1 / 12 * Environment.g_angle_max, c_m=1.0)
        self.goal_angle.zero = trapezoid(-1 / 6 * Environment.g_angle_max, -1 / 12 * Environment.g_angle_max, 1 / 12 * Environment.g_angle_max, 1 / 6 * Environment.g_angle_max, c_m=1.0)
        self.goal_angle.small_pos = trapezoid(1 / 12 * Environment.g_angle_max, 1 / 6 * Environment.g_angle_max, 1 / 3 * Environment.g_angle_max, 5 / 12 * Environment.g_angle_max, c_m=1.0)
        if self.plot_membership_functions:
            self.goal_angle.small_neg.plot()
            plt.show()
            self.goal_angle.zero.plot()
            plt.show()
            self.goal_angle.small_pos.plot()
            plt.show()

        self.goal_angle.large_pos_lin = R(1 / 3 * Environment.g_angle_max, 5 / 12 * Environment.g_angle_max)
        self.goal_angle.large_pos_rect = rectangular(5 / 12 * Environment.g_angle_max, 1 / 2 * Environment.g_angle_max - 1, c_m=1.0)
        self.goal_angle.large_pos = self.goal_angle.large_pos_lin + self.goal_angle.large_pos_rect
        if self.plot_membership_functions:
            self.goal_angle.large_pos.plot()
            plt.show()

    def __create_negative_energy_domains(self):
        """ Creates all five negative energy domains within the field of view of each pedestrian.
        """

        self.negative_energies = {'l': self.__create_negative_energy_domain(),  # left
                                  'fl': self.__create_negative_energy_domain(),  # front left
                                  'f': self.__create_negative_energy_domain(),  # front
                                  'fr': self.__create_negative_energy_domain(),  # front right
                                  'r': self.__create_negative_energy_domain()  # right
                                }

    def __create_negative_energy_domain(self) -> Domain:
        negative_energy = Domain("negative energy", 0, Environment.neg_energy_max, res=0.1)
        
        negative_energy.low_rect = rectangular(0, 0.05, c_m=1.0)
        negative_energy.low_lin = S(0.05, 0.075)
        negative_energy.low = negative_energy.low_rect + negative_energy.low_lin
        if self.plot_membership_functions:
            negative_energy.low.plot()
            plt.show()

        negative_energy.high_lin = R(0.05, 0.075)
        negative_energy.high_rect = rectangular(0.075, Environment.neg_energy_max, c_m=1.0)
        negative_energy.high = negative_energy.high_lin + negative_energy.high_rect
        if self.plot_membership_functions:
            negative_energy.high.plot()
            plt.show()

        return negative_energy


    def get_goal_angle_domain(self) -> Domain:
        return self.goal_angle

    def get_goal_distance_domain(self) -> Domain:
        return self.goal_distance

    def get_velocity_domain(self) -> Domain:
        return self.velocity

    def get_direction_domain(self) -> Domain:
        return self.fov

    def get_distance_domains(self) -> Dict[str, Domain]:
        return self.distances

    def get_negative_energy_domain(self) -> Domain:
        return self.negative_energies