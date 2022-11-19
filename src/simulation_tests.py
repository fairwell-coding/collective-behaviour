from fuzzy_domains import FuzzyDomains
from goal import Goal
from simulation import Simulation


def __test_goal_angle():
    sim = Simulation(FuzzyDomains(), Goal((20.0, 20.0)))
    sim.add_pedestrian((1.0, 1.0), 0.0, 0.0)
    sim.run()


def __test_goal_angle_2():
    sim = Simulation(FuzzyDomains(), Goal((-16.34, -16.34)))
    sim.add_pedestrian((0.0, 0.0), 256.3, 1.0)
    sim.run()


if __name__ == '__main__':
    # __test_goal_angle()
    __test_goal_angle_2()
