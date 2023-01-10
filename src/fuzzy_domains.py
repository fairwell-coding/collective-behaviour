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
        self.__create_occupied_angle_domain()  # {Small, Large} angle
        self.__create_obstacle_impact_domain() # {Small, Middle, Large} impact
        self.__create_collision_risk_domain() # {Low, High} risk

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

        low = 11 / 19
        high = 14 / 19
        distance.near_rect = rectangular(0, low * Environment.dmax, c_m=1.0)
        distance.near_lin = S(low * Environment.dmax, high * Environment.dmax)
        distance.near = distance.near_rect + distance.near_lin
        if self.plot_membership_functions:
            distance.near.plot()
            plt.show()

        distance.far_lin = R(low * Environment.dmax, high * Environment.dmax)
        distance.far_rect = rectangular(high * Environment.dmax, Environment.dmax, c_m=1.0)
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

    def __create_occupied_angle_domain(self):
        self.occupied_angle = Domain("occupied angle", 0, Environment.occupied_g_angle_max, res=0.1)

        self.occupied_angle.small_rect = rectangular(0, 10, c_m=1.0)
        self.occupied_angle.small_lin = S(10, 12.5)
        self.occupied_angle.small = self.occupied_angle.small_rect + self.occupied_angle.small_lin
        if self.plot_membership_functions:
            self.occupied_angle.small.plot()
            plt.show()

        self.occupied_angle.large_lin = R(10, 12.5)
        self.occupied_angle.large_rect = rectangular(12.5, Environment.occupied_g_angle_max, c_m=1.0)
        self.occupied_angle.large = self.occupied_angle.large_lin + self.occupied_angle.large_rect
        if self.plot_membership_functions:
            self.occupied_angle.large.plot()
            plt.show()

    def __create_obstacle_impact_domain(self):
        self.obstacle_impact = Domain("obstacle impact", 0, Environment.oi_max, res=.1)

        self.obstacle_impact.small_rect = rectangular(0, 1 / 6 * Environment.oi_max)
        self.obstacle_impact.small_lin = S(1 / 6 * Environment.oi_max, 1 / 4 * Environment.oi_max)
        self.obstacle_impact.small = self.obstacle_impact.small_rect + self.obstacle_impact.small_lin
        if self.plot_membership_functions:
            self.obstacle_impact.small.plot()
            plt.show()

        self.obstacle_impact.middle = trapezoid(1 / 6 * Environment.oi_max, 1 / 4 * Environment.oi_max, 3 / 4 * Environment.oi_max, 5 / 6 * Environment.oi_max)
        if self.plot_membership_functions:
            self.obstacle_impact.middle.plot()
            plt.show()

        self.obstacle_impact.large_lin = R(3 / 4 * Environment.oi_max, 5 / 6 * Environment.oi_max)
        self.obstacle_impact.large_rect = rectangular(5 / 6 * Environment.oi_max, Environment.oi_max)
        self.obstacle_impact.large = self.obstacle_impact.large_lin + self.obstacle_impact.large_rect
        if self.plot_membership_functions:
            self.obstacle_impact.large.plot()
            plt.show()

    def __create_collision_risk_domain(self):
        self.collision_risk = Domain("collision risk", 0, Environment.cr_max, res=0.1)

        self.collision_risk.low_rect = rectangular(0, 0.4, c_m=1.0)
        self.collision_risk.low_lin = S(0.4, 0.6)
        self.collision_risk.low = self.collision_risk.low_rect + self.collision_risk.low_lin
        if self.plot_membership_functions:
            self.collision_risk.low.plot()
            plt.show()

        self.collision_risk.high_lin = R(0.4, 0.6)
        self.collision_risk.high_rect = rectangular(0.6, Environment.cr_max, c_m=1.0)
        self.collision_risk.high = self.collision_risk.high_lin + self.collision_risk.high_rect
        if self.plot_membership_functions:
            self.collision_risk.high.plot()
            plt.show()

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

    def get_negative_energy_domain(self):
        return self.negative_energies
    
    def get_occupied_angle_domain(self):
        return self.occupied_angle
    
    def get_obstacle_impact_domain(self):
        return self.obstacle_impact

    def get_collision_risk_domain(self):
        return self.collision_risk
    
if __name__ == "__main__":
    t = trapezoid(-1 / 6 * Environment.fov, -1 / 12 * Environment.fov, 1 / 12 * Environment.fov, 1 / 6 * Environment.fov, c_m=1.0)
    print(t)
    # FuzzyDomains(True)