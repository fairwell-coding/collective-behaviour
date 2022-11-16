from typing import Tuple


class Goal:
    def __init__(self, coordinates: Tuple[float, float]) -> None:
        self.coordinates = coordinates

    def get_coordinates(self) -> Tuple[float, float]:
        return self.coordinates
