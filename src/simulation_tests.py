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


def __test_assailant_target_selection():
    sim = Simulation(FuzzyDomains(), Goal((-16.34, -16.34)))
    sim.add_pedestrian((1, 1), 180, 1.0)
    sim.add_pedestrian((2, 1), 180, 1.0)
    sim.add_pedestrian((3, 1), 180, 1.0)
    sim.add_pedestrian((1, 2), 180, 1.0)
    sim.add_pedestrian((2, 2), 180, 1.0)
    sim.add_pedestrian((3, 2), 180, 1.0)
    sim.add_pedestrian((0, -5.5), 180, 1.0)
    sim.add_pedestrian((4, -2), 180, 1.0)
    sim.add_obstacle([(3, -1), (3, -2), (3, -3)])
    sim.add_obstacle([(3, 0), (4, 1)])
    # sim.add_pedestrian((2, 6), 180, 1.0)
    sim.add_assailant((2, -2), 180, 1.0)
    sim.add_assailant((5, 0), 180, 1.0)


    for assailant in sim.get_assailants():
        print(assailant.target_selection_distance(key=min))


def __test_assailant_target_selection_2():
    sim = Simulation(FuzzyDomains(), Goal((-16.34, -16.34)))
    sim.add_pedestrian((0, 5), 180, 1.0)
    sim.add_pedestrian((0, 4), 180, 1.0)
    sim.add_pedestrian((0, 6), 180, 1.0)
    sim.add_pedestrian((4, 7), 180, 1.0)
    sim.add_pedestrian((-1, 5), 180, 1.0)
    sim.add_pedestrian((1, 5), 180, 1.0)
    sim.add_pedestrian((-2, 5), 180, 1.0)
    sim.add_assailant((0, 0), 180, 1.0)


    for assailant in sim.get_assailants():
        print(assailant.target_selection_peripherality(key=max))


if __name__ == '__main__':
    #__test_goal_angle()
    # __test_goal_angle_2()
    #__test_assailant_target_selection()
    __test_assailant_target_selection_2()
