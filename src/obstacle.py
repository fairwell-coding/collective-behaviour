from typing import List, Tuple


class Obstacle:
    def __init__(self, vertices: List[Tuple[float, float]]) -> None:
        self.vertices = vertices

    def get_vertices(self) -> List[Tuple[float, float]]:
        return self.vertices
