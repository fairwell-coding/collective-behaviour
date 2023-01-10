from fuzzy_domains import FuzzyDomains
from goal import Goal
from simulation import Simulation

if __name__ == '__main__':
    # sim = Simulation(FuzzyDomains(), Goal((7.89, 3.21)))
    sim = Simulation(FuzzyDomains(), Goal((100, 100)),scale=10)
    #sim.add_manual_pedestrian((50,10),90)
    #sim.add_pedestrian((74, 63), 360-80, 0.0)
    sim.add_pedestrian((10, 10), 0, 0.0)
    #sim.add_pedestrian((1.0, 21.0), 43.0, 0.0)
    #sim.add_obstacle([(1.0, 4.0), (4.0, 4.0), (4.0, 6.0), (1.0, 6.0)])
    sim.add_obstacle([[0, 80], [120,80]])
    sim.run()
