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
    # sim.add_assailant((2, -2), 180, 1.0)
    # sim.add_assailant((5, 0), 180, 1.0)
    sim.run()

    # for assailant in sim.get_assailants():
    # print(assailant.target_selection_distance(key=min))


def __test_assailant_target_selection_2():
    sim = Simulation(FuzzyDomains(), Goal((-16.34, -16.34)))
    # sim.add_pedestrian((150, 155), 180, 1.0)
    # sim.add_pedestrian((150, 124), 180, 1.0)
    # sim.add_pedestrian((120, 165), 180, 1.0)
    sim.add_pedestrian((14, 57), 180, 1.0)
    sim.add_pedestrian((94, 37), 180, 1.0)
    # sim.add_pedestrian((-31, 25), 180, 1.0)
    # sim.add_pedestrian((111, 5), 180, 1.0)
    # sim.add_pedestrian((-2, 5), 180, 1.0)
    sim.add_pedestrian((66, 56), 180, 1.0)
    sim.add_assailant((50, 75), 180, 1.0, predator_type="Advanced")
    sim.add_assailant((70, 35), 180, 1.0, predator_type="Advanced")
    sim.add_obstacle([(120, 120), (100, 100)])
    sim.add_obstacle([(80, 80), (80, 30), (75, -9)])
    sim.run()

    # for assailant in sim.get_assailants():
    # print(assailant.target_selection_distance(key=min))
    # print(assailant.target_selection_peripherality(key=max))
    # print((assailant.coordinates[0], assailant.coordinates[1]))


def __test_simple_room():
    sim = Simulation(FuzzyDomains(), Goal((-16.34, -16.34)))
    sim.add_obstacle([(20, 80), (20, 40)])
    sim.add_obstacle([(20, 100), (140, 100), (140, 20), (20, 20)])
    sim.add_pedestrian((90, 70), 180, 1.0)
    sim.add_pedestrian((100, 75), 180, 1.0)
    sim.add_pedestrian((80, 80), 180, 1.0)
    sim.add_pedestrian((50, 30), 180, 1.0)
    sim.add_pedestrian((40, 45), 180, 1.0)
    sim.add_pedestrian((60, 55), 180, 1.0)
    sim.add_assailant((30, 30), 180, 1.0, predator_type="Primitive")
    sim.add_assailant((100, 32), 180, 1.0, predator_type="Primitive")
    # sim.add_pedestrian((66, 56), 180, 1.0)
    # sim.add_pedestrian((66, 56), 180, 1.0)
    # sim.add_pedestrian((66, 56), 180, 1.0)
    sim.run()

    # for assailant in sim.get_assailants():
    # print(assailant.target_selection_distance(key=min))
    # print(assailant.target_selection_peripherality(key=max))


def __test_video_scenario():
    sim = Simulation(FuzzyDomains(), Goal((150, 150)), tick_rate=300)
    sim.add_obstacle([(20, 80), (20, 40)])
    sim.add_obstacle([(20, 100), (140, 100), (140, 20), (20, 20)])
    sim.add_assailant((50, 50), 180, 1.0, predator_type="Primitive")
    sim.add_assailant((120, 25), 180, 1.0, predator_type="Primitive")
    sim.add_pedestrian((66, 56), 75, 1.0)
    sim.add_pedestrian((68, 62), 75, 1.0)
    sim.add_pedestrian((80, 78), 120, 1.0)
    sim.add_pedestrian((110, 80), 120, 1.0)
    sim.add_pedestrian((100, 40), 120, 1.0)
    sim.add_pedestrian((120, 110), 120, 1.0)
    sim.run()


def __test_video_scenario2():
    """ Pedestrians navigating obstacle to get to goal.
    """

    sim = Simulation(FuzzyDomains(), Goal((50, 50)), tick_rate=300)
    sim.add_obstacle([(10, 70), (100, 70)])
    sim.add_pedestrian((40, 85), 75, 1.0)
    sim.add_pedestrian((40, 90), 135, 1.0)
    sim.add_pedestrian((45, 80), 135, 1.0)
    sim.run()


def __test_video_scenario2():
    """ Pedestrians navigating obstacle to get to goal.
    """

    sim = Simulation(FuzzyDomains(), Goal((50, 50)), tick_rate=300)
    sim.add_obstacle([(10, 70), (100, 70)])

    # pedestrian group 1
    sim.add_pedestrian((40, 85), 75, 1.0)
    sim.add_pedestrian((40, 90), 135, 1.0)
    sim.add_pedestrian((45, 80), 135, 1.0)

    # pedestrian group 2
    sim.add_pedestrian((80, 85), 75, 1.0)
    sim.add_pedestrian((80, 90), 30, 1.0)
    sim.add_pedestrian((85, 80), 210, 1.0)

    sim.run()


def __test_video_scenario3():
    """ Showcase difference in primitive vs advanced predator behaviour. The primitive predator goes for the nearest pedestrian while the advanced predator goes for the isolated one.
    """

    sim = Simulation(FuzzyDomains(), Goal((50, 50)), tick_rate=1000)
    sim.add_obstacle([(10, 70), (100, 70)])

    # pedestrian group 1
    sim.add_pedestrian((40, 85), 75, 1.0)
    sim.add_pedestrian((40, 90), 135, 1.0)
    sim.add_pedestrian((45, 80), 135, 1.0)

    # primitive predator goes for group 1 since these pedestrians are nearer
    sim.add_assailant((60, 100), 180, 1.0, predator_type="Primitive")

    # pedestrian group 2 -> "isolated victim"
    sim.add_pedestrian((80, 85), 75, 1.0)

    # advanced predator goes for group 2 since group 2 is isolated
    sim.add_assailant((50, 100), 180, 1.0, predator_type="Advanced")

    sim.run()


if __name__ == '__main__':
    # __test_goal_angle()
    # __test_goal_angle_2()
    # __test_assailant_target_selection()
    # __test_assailant_target_selection_2()
    # __test_simple_room()
    # __test_video_scenario()
    # __test_video_scenario2()
    __test_video_scenario3()
