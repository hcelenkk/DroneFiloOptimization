class DeliveryPoint:
    def __init__(self, id: int, pos: tuple, weight: float, priority: int, time_window: tuple):
        self.id = id
        self.pos = pos  # (x, y)
        self.weight = weight  # kg
        self.priority = priority  # 1 (düşük) - 5 (yüksek)
        self.time_window = time_window  # (start, end), e.g., ("09:00", "10:00")