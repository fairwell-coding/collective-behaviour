from typing import List, Tuple


class Simulation:
    def __init__(self) -> None:
        self.people = []
        self.obstacles = []
        self.tick_rate = 60
        
    def update():
        pass
    
class Obstacle:
    def __init__(self) -> None:
        self.vertices: List[Tuple()] = []
        
class Person:
    def __init__(self, position: Tuple[float,float], simulation: Simulation) -> None:
        self.simulation = simulation
        self.position: Tuple[float,float] = position
        self.velocity: float = 0
        self.angle: float = 0
    
    def loc_obstacle_avoiding(self): #Outputs speed and angle
        pass
    
    def path_searching(self):
        pass
    
    def goal_seeking(self):
        pass
        
    def update(self):
        pass
        #Update position

if __name__ == '__main__':
    print('entry point')
