import random
from src.models.drone import Drone
from src.models.delivery_point import DeliveryPoint
from src.models.no_fly_zone import NoFlyZone

def generate_data(num_drones=1, num_deliveries=20, num_no_fly_zones=3):
    drones = [Drone(
        id=i+1,
        start_pos=(random.uniform(0, 50), random.uniform(0, 50)),
        max_weight=random.uniform(8, 12),
        battery=random.uniform(80, 120),
        speed=random.uniform(8, 12),
        charge_time=random.uniform(300, 600)
    ) for i in range(num_drones)]

    deliveries = [DeliveryPoint(
        id=i+1,
        pos=(random.uniform(0, 100), random.uniform(0, 100)),
        weight=random.uniform(1, 3),
        priority=random.randint(1, 5),
        time_window=(random.randint(0, 600), random.randint(600, 1200))
    ) for i in range(num_deliveries)]

    # Kare/Dikdörtgen No-Fly Zone'lar üret
    no_fly_zones = []
    for i in range(num_no_fly_zones):
        # Rastgele merkez noktası (kenarlardan uzak)
        center_x = random.uniform(15, 85)
        center_y = random.uniform(15, 85)
        
        # Rastgele boyutlar
        width = random.uniform(8, 20)  # Genişlik
        height = random.uniform(8, 20)  # Yükseklik
        
        # Kare/dikdörtgenin köşe koordinatlarını hesapla (saat yönünde)
        half_w, half_h = width/2, height/2
        coords = [
            (center_x - half_w, center_y - half_h),  # Sol alt
            (center_x + half_w, center_y - half_h),  # Sağ alt
            (center_x + half_w, center_y + half_h),  # Sağ üst
            (center_x - half_w, center_y + half_h)   # Sol üst
        ]
        
        active_time = (random.randint(0, 600), random.randint(600, 1200))
        no_fly_zones.append(NoFlyZone(id=i+1, coordinates=coords, active_time=active_time))


    for i, zone in enumerate(no_fly_zones):
        coords = zone.coordinates
        center_x = sum(coord[0] for coord in coords) / len(coords)
        center_y = sum(coord[1] for coord in coords) / len(coords)
        width = coords[1][0] - coords[0][0]
        height = coords[2][1] - coords[1][1]
        
    
    return drones, deliveries, no_fly_zones

def generate_square_zones_only(num_no_fly_zones=3):
    """
    Sadece kare şeklinde no-fly zone'lar üretir
    """
    no_fly_zones = []
    for i in range(num_no_fly_zones):
        # Rastgele merkez noktası
        center_x = random.uniform(20, 80)
        center_y = random.uniform(20, 80)
        
        # Kare boyutu (width = height)
        size = random.uniform(10, 25)
        half_size = size / 2
        
        # Kare köşe koordinatları
        coords = [
            (center_x - half_size, center_y - half_size),  # Sol alt
            (center_x + half_size, center_y - half_size),  # Sağ alt
            (center_x + half_size, center_y + half_size),  # Sağ üst
            (center_x - half_size, center_y + half_size)   # Sol üst
        ]
        
        active_time = (random.randint(0, 600), random.randint(600, 1200))
        no_fly_zones.append(NoFlyZone(id=i+1, coordinates=coords, active_time=active_time))
    
    return no_fly_zones

def generate_mixed_shapes(num_no_fly_zones=3):
    """
    Karışık kare ve dikdörtgen şekiller üretir
    """
    no_fly_zones = []
    for i in range(num_no_fly_zones):
        center_x = random.uniform(15, 85)
        center_y = random.uniform(15, 85)
        
        # %50 kare, %50 dikdörtgen
        if random.random() < 0.5:
            # Kare
            size = random.uniform(10, 22)
            width = height = size
        else:
            # Dikdörtgen
            width = random.uniform(12, 25)
            height = random.uniform(8, 18)
        
        half_w, half_h = width/2, height/2
        coords = [
            (center_x - half_w, center_y - half_h),
            (center_x + half_w, center_y - half_h),
            (center_x + half_w, center_y + half_h),
            (center_x - half_w, center_y + half_h)
        ]
        
        active_time = (random.randint(0, 600), random.randint(600, 1200))
        no_fly_zones.append(NoFlyZone(id=i+1, coordinates=coords, active_time=active_time))
    
    return no_fly_zones

if __name__ == "__main__":
    drones, deliveries, no_fly_zones = generate_data(10, 50, 5)
    #print(f"Generated {len(drones)} drones, {len(deliveries)} deliveries, {len(no_fly_zones)} no-fly zones")
    
    #print("\nNo-fly zone koordinatları:")
    #for zone in no_fly_zones:
        #print(f"Zone {zone.id}: {zone.coordinates}")