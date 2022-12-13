from fuzzy_domains import FuzzyDomains
from goal import Goal
from simulation import Simulation

if __name__ == '__main__':
    # sim = Simulation(FuzzyDomains(), Goal((7.89, 3.21)))
    sim = Simulation(FuzzyDomains(), Goal((50, 50)))
    sim.add_pedestrian((1.31, 2.09), 43.2, 0.0)
    sim.add_pedestrian((1.0, 21.0), 43.0, 0.0)
    #sim.add_obstacle([(1.0, 4.0), (4.0, 4.0), (4.0, 6.0), (1.0, 6.0)])
    #sim.add_obstacle([[19.23, 63.78], [36.72, 71.65], [47.84, 61.7], [68.91, 73.98], [75.39, 44.27], [99.12, 79.45], [113.89, 29.0]])
    sim.run()
