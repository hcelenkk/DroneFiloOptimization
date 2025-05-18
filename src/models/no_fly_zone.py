class NoFlyZone:
    def __init__(self, id: int, coordinates: list, active_time: tuple):
        self.id = id
        self.coordinates = coordinates  # [(x1, y1), (x2, y2), ...]
        self.active_time = active_time  # (start, end), e.g., ("09:30", "11:00")