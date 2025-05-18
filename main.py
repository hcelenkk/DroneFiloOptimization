import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from src.utils.data_generator import generate_data
from src.utils.graph import Graph
from src.utils.visualization import plot_routes
from src.algorithms.a_star import AStar
from src.algorithms.csp import CSP
from src.algorithms.genetic_algorithm import GeneticAlgorithm

def main():
    # Test senaryosu: 5 drone, 20 teslimat, 2 uçuş yasağı bölgesi
    drones, delivery_points, no_fly_zones = generate_data(5, 20, 2)
    
    # Grafik oluştur
    graph = Graph(drones, delivery_points, no_fly_zones)
    
    # A* algoritmasını test et
    a_star = AStar(graph)
    start_node = "drone_0"
    goal_node = "dp_0"
    drone = drones[0]
    path, cost = a_star.find_path(start_node, goal_node, drone)
    
    print(f"A* Yolu ({start_node} -> {goal_node}):")
    if path:
        print(f"Yol: {' -> '.join(path)}")
        print(f"Toplam Maliyet: {cost:.2f}")
    else:
        print("Yol bulunamadı.")
    
    # CSP'yi test et
    csp = CSP(drones, delivery_points, graph)
    assignments = csp.solve(current_time="10:00")
    
    print("\nCSP Atamaları:")
    if assignments:
        for drone_id, dp_id in assignments.items():
            print(f"Drone {drone_id} -> Teslimat Noktası {dp_id}")
    else:
        print("Geçerli atama bulunamadı.")
    
    # Genetik Algoritma'yı test et
    ga = GeneticAlgorithm(drones, delivery_points, graph)
    best_routes, best_fitness = ga.run(current_time="10:00")
    
    print("\nGenetik Algoritma Sonuçları:")
    print(f"En İyi Fitness: {best_fitness:.2f}")
    for drone_idx, route in enumerate(best_routes):
        if route:
            print(f"Drone {drone_idx}: {' -> '.join([f'dp_{dp_id}' for dp_id in route])}")
        else:
            print(f"Drone {drone_idx}: Atama yok")
    
    # Rotları görselleştir
    print("\nRotalar görselleştiriliyor, output/routes.png dosyasına kaydedilecek...")
    plot_routes(drones, delivery_points, no_fly_zones, best_routes)
    print("Görselleştirme tamamlandı.")

if __name__ == "__main__":
    main()