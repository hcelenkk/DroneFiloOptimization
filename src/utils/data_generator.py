import random
from src.models.drone import Drone
from src.models.delivery_point import DeliveryPoint
from src.models.no_fly_zone import NoFlyZone

def generate_data(num_drones: int, num_deliveries: int, num_no_fly_zones: int, area_size: int = 1000):
    drones = []
    delivery_points = []
    no_fly_zones = []

    # Drone'lar oluştur
    for i in range(num_drones):
        drone = Drone(
            id=i,
            max_weight=random.uniform(5.0, 20.0),
            battery=random.randint(5000, 20000),
            speed=random.uniform(5.0, 15.0),
            start_pos=(random.uniform(0, area_size), random.uniform(0, area_size))
        )
        drones.append(drone)

    # Teslimat noktaları oluştur
    for i in range(num_deliveries):
        delivery = DeliveryPoint(
            id=i,
            pos=(random.uniform(0, area_size), random.uniform(0, area_size)),
            weight=random.uniform(1.0, 10.0),
            priority=random.randint(1, 5),
            time_window=("09:00", "17:00")  # Sabit zaman aralığı (ileride dinamik yapılabilir)
        )
        delivery_points.append(delivery)

    # Uçuş yasağı bölgeleri oluştur
    for i in range(num_no_fly_zones):
        # Basit bir dörtgen oluştur
        center_x = random.uniform(100, area_size - 100)
        center_y = random.uniform(100, area_size - 100)
        size = random.uniform(50, 200)
        coordinates = [
            (center_x - size, center_y - size),
            (center_x + size, center_y - size),
            (center_x + size, center_y + size),
            (center_x - size, center_y + size)
        ]
        no_fly_zone = NoFlyZone(
            id=i,
            coordinates=coordinates,
            active_time=("09:30", "11:00")  # Sabit zaman aralığı
        )
        no_fly_zones.append(no_fly_zone)

    return drones, delivery_points, no_fly_zones