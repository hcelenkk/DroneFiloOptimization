class NoFlyZone:
    def __init__(self, id: int, coordinates: list, active_time: tuple):
        self.id = id
        self.coordinates = coordinates
        self.active_time = active_time  # (start_time, end_time) in minutes

    def is_active(self, current_time: int) -> bool:
        """Uçuş yasağı bölgesinin belirli bir zamanda aktif olup olmadığını kontrol eder."""
        start_time, end_time = self.active_time
        return start_time <= current_time <= end_time