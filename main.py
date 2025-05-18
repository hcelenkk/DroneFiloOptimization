import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from src.utils.data_generator import generate_data
from src.utils.graph import Graph
from src.algorithms.a_star import AStar

def main():
    # Test senaryosu: 5 drone, 20 teslimat, 2 uçuş yasağı bölgesi
    drones, delivery_points, no_fly_zones = generate_data(5, 20, 2)
    
    # Grafik oluştur
    graph = Graph(drones, delivery_points, no_fly_zones)
    
    # A* algoritmasını test et
    a_star = AStar(graph)
    start_node = "drone_0"
    goal_node = "dp_0"
    drone = drones[0]  # İlk drone'u kullan
    
    path, cost = a_star.find_path(start_node, goal_node, drone)
    
    print(f"A* Yolu ({start_node} -> {goal_node}):")
    if path:
        print(f"Yol: {' -> '.join(path)}")
        print(f"Toplam Maliyet: {cost:.2f}")
    else:
        print("Yol bulunamadı.")

if __name__ == "__main__":
    main()