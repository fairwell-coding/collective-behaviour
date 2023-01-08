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
    sim = Simulation(FuzzyDomains(), Goal((80.34, 116.34)))
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
    sim.add_pedestrian((2, 6), 180, 1.0)
    #sim.add_assailant((2, -2), 180, 1.0)
    #sim.add_assailant((5, 0), 180, 1.0)
    sim.run()


    #for assailant in sim.get_assailants():
        #print(assailant.target_selection_distance(key=min))


def __test_assailant_target_selection_2():
    sim = Simulation(FuzzyDomains(), Goal((-16.34, -16.34)))
    #sim.add_pedestrian((150, 155), 180, 1.0)
    #sim.add_pedestrian((150, 124), 180, 1.0)
    #sim.add_pedestrian((120, 165), 180, 1.0)
    sim.add_pedestrian((14, 57), 180, 1.0)
    sim.add_pedestrian((94, 37), 180, 1.0)
    #sim.add_pedestrian((-31, 25), 180, 1.0)
    #sim.add_pedestrian((111, 5), 180, 1.0)
    #sim.add_pedestrian((-2, 5), 180, 1.0)
    sim.add_pedestrian((66, 56), 180, 1.0)
    sim.add_assailant((50, 75), 180, 1.0)
    sim.add_assailant((70, 35), 180, 1.0)
    sim.add_obstacle([(120, 120), (100, 100)])
    sim.add_obstacle([(80, 80), (80, 30), (75, -9)])
    sim.run()

    #for assailant in sim.get_assailants():
        #print(assailant.target_selection_distance(key=min))
        #print(assailant.target_selection_peripherality(key=max))
        #print((assailant.coordinates[0], assailant.coordinates[1]))


if __name__ == '__main__':
    #__test_goal_angle()
    # __test_goal_angle_2()
    #__test_assailant_target_selection()
    __test_assailant_target_selection_2()
