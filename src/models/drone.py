class Drone:
    def __init__(self, id: int, start_pos: tuple, max_weight: float, battery: float, speed: float, charge_time: float = 300):
        self.id = id
        self.start_pos = start_pos
        self.max_weight = max_weight
        self.battery = battery
        self.speed = speed
        self.charge_time = charge_time  # Şarj süresi (saniye)

    def consume_battery(self, distance: float):
        """Batarya tüketimini hesapla."""
        consumption = distance * (5 / self.speed)  # Basit bir tüketim modeli
        if consumption > self.battery:
            return False
        self.battery -= consumption
        return True

    def charge(self):
        """Bataryayı şarj et."""
        if self.battery < 20:  # %20 altında şarj
            self.battery = 100  # Tam şarj
            return self.charge_time
        return 0