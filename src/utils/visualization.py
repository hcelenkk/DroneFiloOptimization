import matplotlib
matplotlib.use('Agg')  # Tkinter yerine Agg arka ucunu kullan
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from typing import List, Tuple
from src.models.drone import Drone
from src.models.delivery_point import DeliveryPoint
from src.models.no_fly_zone import NoFlyZone

def plot_routes(drones: List[Drone], delivery_points: List[DeliveryPoint], 
                no_fly_zones: List[NoFlyZone], routes: List[List[int]], 
                filename: str = "output/routes.png"):
    """Drone rotalarını ve uçuş yasağı bölgelerini görselleştirir."""
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Uçuş yasağı bölgelerini çiz (gri dolgulu çokgenler)
    for nfz in no_fly_zones:
        polygon = patches.Polygon(nfz.coordinates, closed=True, facecolor='gray', alpha=0.5)
        ax.add_patch(polygon)
    
    # Teslimat noktalarını çiz (kırmızı noktalar)
    for dp in delivery_points:
        ax.plot(dp.pos[0], dp.pos[1], 'ro', label='Teslimat Noktası' if dp == delivery_points[0] else "")
        ax.text(dp.pos[0], dp.pos[1], f'dp_{dp.id}', fontsize=8, ha='right')
    
    # Drone başlangıç pozisyonlarını çiz (mavi noktalar)
    for drone in drones:
        ax.plot(drone.start_pos[0], drone.start_pos[1], 'bo', label='Drone' if drone == drones[0] else "")
        ax.text(drone.start_pos[0], drone.start_pos[1], f'drone_{drone.id}', fontsize=8, ha='left')
    
    # Rotları çiz (her drone için farklı renk)
    colors = ['green', 'blue', 'purple', 'orange', 'cyan']
    for drone_idx, route in enumerate(routes):
        if not route:
            continue
        # Drone başlangıç pozisyonundan başla
        current_pos = drones[drone_idx].start_pos
        path_x = [current_pos[0]]
        path_y = [current_pos[1]]
        
        # Teslimat noktalarına sırayla git
        for dp_id in route:
            dp = next((dp for dp in delivery_points if dp.id == dp_id), None)
            if dp:
                path_x.append(dp.pos[0])
                path_y.append(dp.pos[1])
                current_pos = dp.pos
        
        # Rotayı çiz
        ax.plot(path_x, path_y, color=colors[drone_idx % len(colors)], 
                linestyle='-', linewidth=2, label=f'Drone {drone_idx} Rotası')
    
    # Grafik ayarları
    ax.set_xlabel('X Koordinatı')
    ax.set_ylabel('Y Koordinatı')
    ax.set_title('Drone Teslimat Rotaları')
    ax.legend()
    ax.grid(True)
    
    # PNG olarak kaydet
    plt.savefig(filename, bbox_inches='tight')
    plt.close()