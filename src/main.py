from fuzzy_domains import FuzzyDomains
from goal import Goal
from simulation import Simulation

if __name__ == '__main__':
    sim = Simulation(FuzzyDomains(), Goal((7.89, 3.21)))
    sim.add_pedestrian((1.31, 2.09), 43.2, 0.0)
    sim.add_obstacle([(1.0, 4.0), (4.0, 4.0), (4.0, 6.0), (1.0, 6.0)])
    sim.run()
