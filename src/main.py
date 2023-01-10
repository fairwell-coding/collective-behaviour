from fuzzy_domains import FuzzyDomains
from goal import Goal
from simulation import Simulation

def basic_no_obstacle(**kwargs):
    sim = Simulation(FuzzyDomains(), Goal((110, 110)), scale=7)
    sim.add_manual_pedestrian((50,10),90)
    sim.add_pedestrian((74, 63), 360-80, 0.0)
    sim.add_pedestrian((10, 10), 0, 0.0)
    sim.add_pedestrian((20, 20), 0, 0.0)
    sim.add_pedestrian((30, 30), 0, 0.0)
    sim.add_pedestrian((40, 50), 0, 0.0)
    sim.run(**kwargs)
def run_large_room():
    sim = Simulation(FuzzyDomains(), Goal((110, 110)), scale=7)
    sim.add_obstacle([(20, 80), (20, 40)])
    sim.add_obstacle([(20, 100), (140, 100), (140, 20), (20, 20)])
    sim.add_pedestrian((90, 70), 180, 1.0)
    sim.add_pedestrian((100, 75), 180, 1.0)
    sim.add_pedestrian((80, 80), 180, 1.0)
    sim.add_pedestrian((50, 30), 180, 1.0)
    sim.add_pedestrian((40, 45), 180, 1.0)
    sim.add_pedestrian((60, 55), 180, 1.0)
    sim.add_assailant((30, 30), 180, 1.0, predator_type="Advanced")
    sim.add_assailant((100, 32), 180, 1.0, predator_type="Primitive")
    sim.run(draw_rays=False)

def run_report_simulation(**kwargs):
    sim = Simulation(FuzzyDomains(), Goal((40, 40)), scale=22)
    sim.add_obstacle([(7, 28), (7, 10), (35, 10), (35, 28), (14, 28)])
    sim.add_pedestrian((10, 10), 180, 1.0)
    sim.add_pedestrian((13, 11), 180, 1.0)
    sim.add_pedestrian((16, 7), 180, 1.0)
    sim.add_pedestrian((40, 10), 180, 1.0)
    sim.add_assailant((60, 18), 180, 1.0)
    sim.run(**kwargs)

def run_large_groups_simulation(**kwargs):
    sim = Simulation(FuzzyDomains(), Goal((245, 194)), scale=4)
    sim.add_pedestrian((30, 35), 90, 1.0)
    sim.add_pedestrian((50, 35), 180, 1.0)
    sim.add_pedestrian((70, 35), 180, 1.0)
    sim.add_pedestrian((90, 35), 80, 1.0)
    sim.add_pedestrian((110, 45), 180, 1.0)

    sim.add_pedestrian((240, 43), 180, 1.0)
    sim.add_pedestrian((240, 39), 180, 1.0)
    sim.add_pedestrian((240, 39), 180, 1.0)
    sim.add_pedestrian((245, 43), 180, 1.0)
    sim.add_pedestrian((222, 43), 180, 1.0)

    sim.add_pedestrian((179, 250), 180, 1.0)
    sim.add_pedestrian((135, 240), 180, 1.0)
    sim.add_pedestrian((112, 230), 180, 1.0)
    sim.add_pedestrian((229, 230), 180, 1.0)
    sim.add_pedestrian((200, 220), 180, 1.0)

    sim.add_pedestrian((392, 115), 180, 1.0)
    sim.add_pedestrian((379, 138), 180, 1.0)
    sim.add_pedestrian((332, 195), 180, 1.0)
    sim.add_pedestrian((392, 145), 180, 1.0)

    sim.add_assailant((200, 200), 180, 1.0, predator_type="Primitive")
    sim.add_assailant((240, 240), 180, 1.0, predator_type="Primitive")
    sim.add_assailant((205, 200), 180, 1.0, predator_type="Advanced")
    sim.run(**kwargs)

if __name__ == '__main__':
    run_large_groups_simulation(draw_rays=False)