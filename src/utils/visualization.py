import matplotlib
matplotlib.use('Agg')  # Tkinter yerine Agg arka ucunu kullan
import matplotlib.pyplot as plt
from src.models.drone import Drone
from src.models.delivery_point import DeliveryPoint
from src.models.no_fly_zone import NoFlyZone
from src.utils.graph import Graph

def plot_routes(drones: list[Drone], delivery_points: list[DeliveryPoint], no_fly_zones: list[NoFlyZone], routes: list[list[int]], graph: Graph, filename: str = "output/routes.png"):
    plt.figure(figsize=(10, 10))

    # Uçuş yasağı bölgelerini çiz
    for i, nfz in enumerate(no_fly_zones):
        coords = nfz.coordinates + [nfz.coordinates[0]]  # Poligonu kapatmak için ilk koordinatı ekle
        x, y = zip(*coords)
        plt.fill(x, y, "r", alpha=0.3, label="Uçuş Yasağı Bölgesi" if i == 0 else None)

    # Teslimat noktalarını çiz
    for dp in delivery_points:
        plt.plot(dp.pos[0], dp.pos[1], "bo", label="Teslimat Noktası" if dp == delivery_points[0] else "")
        plt.text(dp.pos[0], dp.pos[1], f"DP{dp.id}")

    # Droneları çiz
    for drone in drones:
        plt.plot(drone.start_pos[0], drone.start_pos[1], "g^", label="Drone" if drone == drones[0] else "")
        plt.text(drone.start_pos[0], drone.start_pos[1], f"D{drone.id}")

    # Rotaları çiz
    colors = ["b", "g", "c", "m", "y", "k"]
    for i, route in enumerate(routes):
        if route:
            color = colors[i % len(colors)]
            x, y = [drones[i].start_pos[0]], [drones[i].start_pos[1]]
            for dp_id in route:
                try:
                    dp = next(dp for dp in delivery_points if dp.id == dp_id)
                    x.append(dp.pos[0])
                    y.append(dp.pos[1])
                except StopIteration:
                    print(f"Uyarı: Teslimat noktası ID {dp_id} bulunamadı, rotada atlanıyor.")
                    continue
            plt.plot(x, y, color=color, linestyle="-", label=f"Drone {drones[i].id} Rotası")

    plt.legend()
    plt.grid(True)
    plt.xlabel("X Koordinatı")
    plt.ylabel("Y Koordinatı")
    plt.title("Drone Rotaları ve Uçuş Yasağı Bölgeleri")
    plt.savefig(filename)
    plt.close()