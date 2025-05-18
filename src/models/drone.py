class Drone:
    def __init__(self, id: int, max_weight: float, battery: int, speed: float, start_pos: tuple):
        self.id = id
        self.max_weight = max_weight  # kg
        self.battery = battery  # mAh
        self.speed = speed  # m/s
        self.start_pos = start_pos  # (x, y)
        self.current_load = 0.0  # Şu anki yük
        self.current_battery = battery  # Kalan batarya