import time
import math
from src.utils.graph import Graph
from src.algorithms.a_star import AStar
from src.algorithms.csp import CSP
from src.algorithms.genetic_algorithm import GeneticAlgorithm
from src.utils.visualization import plot_routes
from src.utils.data_generator import generate_data
from src.models.drone import Drone
from src.models.delivery_point import DeliveryPoint
from src.models.no_fly_zone import NoFlyZone

def print_separator(title):
    print("\n" + "="*60)
    print(f"  {title}")
    print("="*60)

def print_subsection(title):
    print(f"\n--- {title} ---")

def calculate_cost(distance, weight, priority):

    priority_penalty = (6 - priority) * 100 
    return distance * weight + priority_penalty

def calculate_heuristic(current_pos, target_pos, no_fly_zones, current_time=0):
    distance = math.sqrt((target_pos[0] - current_pos[0])**2 + (target_pos[1] - current_pos[1])**2)

    no_fly_penalty = 0
    for zone in no_fly_zones:
        if is_in_no_fly_zone(current_pos, zone, current_time):
            no_fly_penalty += 500  
        elif is_near_no_fly_zone(current_pos, zone, 10): 
            no_fly_penalty += 100 
    
    return distance + no_fly_penalty

def is_in_no_fly_zone(pos, zone, current_time):
    # Zaman kontrolü
    if hasattr(zone, 'active_time'):
        start_time, end_time = zone.active_time
        if not (start_time <= current_time <= end_time):
            return False
    
    # Koordinat kontrolü
    if hasattr(zone, 'coordinates') and zone.coordinates:
        coords = zone.coordinates
        min_x = min(coord[0] for coord in coords)
        max_x = max(coord[0] for coord in coords)
        min_y = min(coord[1] for coord in coords)
        max_y = max(coord[1] for coord in coords)
        
        return min_x <= pos[0] <= max_x and min_y <= pos[1] <= max_y
    
    return False

def is_near_no_fly_zone(pos, zone, threshold):
    if hasattr(zone, 'coordinates') and zone.coordinates:
        coords = zone.coordinates
        center_x = sum(coord[0] for coord in coords) / len(coords)
        center_y = sum(coord[1] for coord in coords) / len(coords)
        distance = math.sqrt((pos[0] - center_x)**2 + (pos[1] - center_y)**2)
        return distance <= threshold
    return False

def calculate_ga_fitness(route, drone, deliveries, total_energy, constraint_violations):
    delivery_count = len(route) if route else 0
    fitness = (delivery_count * 50) - (total_energy * 0.1) - (constraint_violations * 1000)
    return fitness

def analyze_graph_structure(graph, drones, deliveries, no_fly_zones):
    print_subsection("Graf Yapısı Analizi")
    
    # Düğüm sayısı 
    total_nodes = len(drones) + len(deliveries)
    print(f"Toplam Düğüm Sayısı: {total_nodes}")
    print(f"  - Drone Düğümleri: {len(drones)}")
    print(f"  - Teslimat Düğümleri: {len(deliveries)}")
    
    # Teorik kenar sayısı 
    theoretical_edges = len(drones) * len(deliveries)
    print(f"Teorik Kenar Sayısı: {theoretical_edges}")
    
    # No-fly zone etkisi
    active_zones = len(no_fly_zones)
    print(f"Aktif No-Fly Zone Sayısı: {active_zones}")
    
    # Maliyet fonksiyonu örneği
    print(f"\nMaliyet Fonksiyonu: Maliyet = Mesafe × Ağırlık + (Öncelik × 100)")
    if deliveries:
        sample_delivery = deliveries[0]
        sample_distance = 10.0
        sample_cost = calculate_cost(sample_distance, sample_delivery.weight, sample_delivery.priority)
        print(f"  - Mesafe: {sample_distance}, Ağırlık: {sample_delivery.weight:.1f}, Öncelik: {sample_delivery.priority}")
        print(f"  - Maliyet: {sample_cost:.2f}")

def evaluate_astar_performance(drones, deliveries, no_fly_zones, graph):
    print_subsection("A* Algoritması Analizi")
    
    a_star = AStar(graph)
    
    total_time = 0
    successful_paths = 0
    failed_paths = 0
    
    # Her drone için ilk teslimat noktasına giden yolu test et
    for i, drone in enumerate(drones[:min(5, len(drones))]):  # İlk 5 drone test et
        if i < len(deliveries):
            target_delivery = deliveries[i]
            
            start_time = time.time()
            try:
                path, cost = a_star.find_path(f"drone_{drone.id}", f"dp_{target_delivery.id}", drone)
                elapsed_time = time.time() - start_time
                total_time += elapsed_time
                
                if path:
                    successful_paths += 1
                    print(f"Drone {drone.id} → DP {target_delivery.id}: Başarılı (Maliyet: {cost:.2f}, Süre: {elapsed_time:.4f}s)")
                    
                    # Kapasite kontrolü
                    if target_delivery.weight > drone.max_weight:
                        print(f"  ⚠️  Kapasite aşımı! Ağırlık: {target_delivery.weight:.1f} > Max: {drone.max_weight:.1f}")
                else:
                    failed_paths += 1
                    print(f"Drone {drone.id} → DP {target_delivery.id}: Başarısız")
                    
            except Exception as e:
                failed_paths += 1
                elapsed_time = time.time() - start_time
                total_time += elapsed_time
                print(f"Drone {drone.id} → DP {target_delivery.id}: Hata - {str(e)}")
    
    avg_time = total_time / max(1, successful_paths + failed_paths)
    print(f"\nA* Özet:")
    print(f"  - Başarılı Yollar: {successful_paths}")
    print(f"  - Başarısız Yollar: {failed_paths}")
    print(f"  - Ortalama Süre: {avg_time:.4f} saniye")
    print(f"  - Başarı Oranı: {(successful_paths/(successful_paths+failed_paths)*100) if (successful_paths+failed_paths) > 0 else 0:.1f}%")
    
    return total_time, successful_paths, failed_paths

def evaluate_csp_performance(drones, deliveries, no_fly_zones):

    print_subsection("CSP Analizi")
    
    csp = CSP(drones, deliveries, no_fly_zones)
        
    start_time = time.time()
    try:
        assignments = csp.solve(current_time="00:00")
        csp_time = time.time() - start_time
        
        if assignments:
            print(f"\nCSP Çözüm Bulundu! (Süre: {csp_time:.4f} saniye)")
            
            # Kısıt ihlallerini kontrol et
            constraint_violations = 0
            assigned_deliveries = set()
            
            for drone_id, delivery_ids in assignments.items():
                if delivery_ids:
                    # Duplicate teslimat kontrolü
                    for dp_id in delivery_ids:
                        if dp_id in assigned_deliveries:
                            constraint_violations += 1
                            
                        assigned_deliveries.add(dp_id)
                    
                    # Kapasite kontrolü
                    drone = next((d for d in drones if d.id == drone_id), None)
                    if drone:
                        total_weight = sum(next(d.weight for d in deliveries if d.id == dp_id) for dp_id in delivery_ids)
                        if total_weight > drone.max_weight:
                            constraint_violations += 1
                            print(f"  ⚠️  Kapasite aşımı Drone {drone_id}: {total_weight:.1f} > {drone.max_weight:.1f}")
            
            assigned_count = len(assigned_deliveries)
            assignment_rate = (assigned_count / len(deliveries)) * 100
            
            print(f"  - Atanan Teslimat: {assigned_count}/{len(deliveries)} ({assignment_rate:.1f}%)")
            print(f"  - Kısıt İhlali: {constraint_violations}")
            
        else:
            print(f"\nCSP Çözüm Bulunamadı! (Süre: {csp_time:.4f} saniye)")
            constraint_violations = len(deliveries)  # Tüm teslimatlar başarısız
            
    except Exception as e:
        csp_time = time.time() - start_time
        print(f"\nCSP Hatası: {str(e)} (Süre: {csp_time:.4f} saniye)")
        constraint_violations = len(deliveries)
        assignments = None
    
    return csp_time, assignments, constraint_violations

def evaluate_ga_performance(drones, deliveries, no_fly_zones, graph):
    """Genetic Algorithm performansını değerlendir"""
    print_subsection("Genetik Algoritma Analizi")
    
    ga = GeneticAlgorithm(drones, deliveries, graph)
    
    print("GA Parametreleri:")
    print(f"  - Popülasyon Boyutu: {getattr(ga, 'population_size', 'Bilinmiyor')}")
    print(f"  - Nesil Sayısı: {getattr(ga, 'generations', 'Bilinmiyor')}")
    
    print("\nFitness Fonksiyonu:")
    print("  Fitness = (Teslimat Sayısı × 50) - (Toplam Enerji × 0.1) - (İhlal × 1000)")
    
    start_time = time.time()
    try:
        best_routes, best_fitness = ga.run(current_time="00:00")
        ga_time = time.time() - start_time
        
        print(f"\nGA Tamamlandı! (Süre: {ga_time:.4f} saniye)")
        print(f"En İyi Fitness: {best_fitness:.2f}")
        
        # Route analizi
        total_deliveries = 0
        total_energy = 0
        constraint_violations = 0
        
        for i, route in enumerate(best_routes):
            if route:
                total_deliveries += len(route)
                
                # Enerji hesaplama (basit)
                drone = drones[i] if i < len(drones) else None
                if drone:
                    route_energy = len(route) * 10  # Basit hesaplama
                    total_energy += route_energy
                    
                    # Kapasite kontrolü
                    route_weight = sum(next(d.weight for d in deliveries if d.id == dp_id) for dp_id in route if any(d.id == dp_id for d in deliveries))
                    if route_weight > drone.max_weight:
                        constraint_violations += 1
        
        # Fitness doğrulama
        calculated_fitness = calculate_ga_fitness(None, None, deliveries, total_energy, constraint_violations)
        calculated_fitness = (total_deliveries * 50) - (total_energy * 0.1) - (constraint_violations * 1000)
        
        print(f"Fitness Detayları:")
        print(f"  - Teslimat Sayısı: {total_deliveries} × 50 = {total_deliveries * 50}")
        print(f"  - Enerji Cezası: {total_energy:.1f} × 0.1 = {total_energy * 0.1:.1f}")
        print(f"  - Kısıt İhlali: {constraint_violations} × 1000 = {constraint_violations * 1000}")
        print(f"  - Hesaplanan Fitness: {calculated_fitness:.2f}")
        
        completion_rate = (total_deliveries / len(deliveries)) * 100 if len(deliveries) > 0 else 0
        print(f"  - Tamamlanma Oranı: {completion_rate:.1f}%")
        
    except Exception as e:
        ga_time = time.time() - start_time
        print(f"\nGA Hatası: {str(e)} (Süre: {ga_time:.4f} saniye)")
        best_routes = []
        best_fitness = -float('inf')
    
    return ga_time, best_routes, best_fitness

def run_scenario(scenario_name, num_drones, num_deliveries, num_no_fly_zones, use_fixed_data=False, fixed_data=None):
    """Test senaryosunu çalıştır"""
    print_separator(f"SENARYO: {scenario_name}")
    print(f"Parametreler: {num_drones} drone, {num_deliveries} teslimat, {num_no_fly_zones} no-fly zone")
    
    # Veri üretimi
    if use_fixed_data and fixed_data:
        drones = fixed_data['drones']
        deliveries = fixed_data['deliveries']
        no_fly_zones = fixed_data['no_fly_zones']
    else:
        drones, deliveries, no_fly_zones = generate_data(num_drones, num_deliveries, num_no_fly_zones)
    
    graph = Graph(drones, deliveries, no_fly_zones)
    
    # Graf analizi
    analyze_graph_structure(graph, drones, deliveries, no_fly_zones)
    
    # Algoritma performansları
    astar_time, successful_paths, failed_paths = evaluate_astar_performance(drones, deliveries, no_fly_zones, graph)
    csp_time, csp_assignments, csp_violations = evaluate_csp_performance(drones, deliveries, no_fly_zones)
    ga_time, ga_routes, ga_fitness = evaluate_ga_performance(drones, deliveries, no_fly_zones, graph)
    
    # Özet
    print_subsection("SENARYO ÖZETİ")
    print(f"Çalışma Süreleri:")
    print(f"  - A*: {astar_time:.4f} saniye")
    print(f"  - CSP: {csp_time:.4f} saniye") 
    print(f"  - GA: {ga_time:.4f} saniye")
    
    # Tamamlanma oranları
    astar_completion = (successful_paths / len(drones)) * 100 if len(drones) > 0 else 0
    csp_completion = ((len(deliveries) - csp_violations) / len(deliveries)) * 100 if len(deliveries) > 0 else 0
    ga_completion = (sum(len(route) if route else 0 for route in ga_routes) / len(deliveries)) * 100 if len(deliveries) > 0 else 0
    
    print(f"Tamamlanma Oranları:")
    print(f"  - A*: {astar_completion:.1f}%")
    print(f"  - CSP: {csp_completion:.1f}%")
    print(f"  - GA: {ga_completion:.1f}%")
    
    # Enerji tüketimi (tahmini)
    avg_energy = sum(d.battery * 0.1 for d in drones) / len(drones)
    print(f"Ortalama Enerji Tüketimi: {avg_energy:.2f} mAh")
    
    # Performans değerlendirmesi
    if ga_time < 60:  # 1 dakika altı
        print("✅ Performans Hedefi: BAŞARILI (< 1 dakika)")
    else:
        print("❌ Performans Hedefi: BAŞARISIZ (> 1 dakika)")
    
    return {
        'drones': drones,
        'deliveries': deliveries, 
        'no_fly_zones': no_fly_zones,
        'ga_routes': ga_routes,
        'graph': graph
    }

def main():
    print_separator("DRONE TESLİMAT OPTİMİZASYONU - PERFORMANS ANALİZİ")
    
    # Senaryo 1 için sabit veri seti
    scenario1_drones = [
        Drone(id=1, start_pos=(10, 10), max_weight=4.0, battery=12000, speed=8.0),
        Drone(id=2, start_pos=(20, 30), max_weight=3.5, battery=10000, speed=10.0),
        Drone(id=3, start_pos=(50, 50), max_weight=5.0, battery=15000, speed=7.0),
        Drone(id=4, start_pos=(80, 20), max_weight=2.0, battery=8000, speed=12.0),
        Drone(id=5, start_pos=(40, 70), max_weight=6.0, battery=20000, speed=5.0),
    ]
    scenario1_deliveries = [
        DeliveryPoint(id=1, pos=(15, 25), weight=1.5, priority=3, time_window=(0, 60)),
        DeliveryPoint(id=2, pos=(30, 40), weight=2.0, priority=5, time_window=(0, 30)),
        DeliveryPoint(id=3, pos=(70, 80), weight=3.0, priority=2, time_window=(20, 80)),
        DeliveryPoint(id=4, pos=(90, 10), weight=1.0, priority=4, time_window=(10, 40)),
        DeliveryPoint(id=5, pos=(45, 60), weight=4.0, priority=1, time_window=(30, 90)),
        DeliveryPoint(id=6, pos=(25, 15), weight=2.5, priority=3, time_window=(0, 50)),
        DeliveryPoint(id=7, pos=(60, 30), weight=1.0, priority=5, time_window=(5, 25)),
        DeliveryPoint(id=8, pos=(85, 90), weight=3.5, priority=2, time_window=(40, 100)),
        DeliveryPoint(id=9, pos=(10, 80), weight=2.0, priority=4, time_window=(15, 45)),
        DeliveryPoint(id=10, pos=(95, 50), weight=1.5, priority=3, time_window=(0, 60)),
        DeliveryPoint(id=11, pos=(55, 20), weight=0.5, priority=5, time_window=(0, 20)),
        DeliveryPoint(id=12, pos=(35, 75), weight=2.0, priority=1, time_window=(50, 120)),
        DeliveryPoint(id=13, pos=(75, 40), weight=3.0, priority=3, time_window=(10, 50)),
        DeliveryPoint(id=14, pos=(20, 90), weight=1.5, priority=4, time_window=(30, 70)),
        DeliveryPoint(id=15, pos=(65, 65), weight=4.5, priority=2, time_window=(25, 75)),
        DeliveryPoint(id=16, pos=(40, 10), weight=2.0, priority=5, time_window=(0, 30)),
        DeliveryPoint(id=17, pos=(5, 50), weight=1.0, priority=3, time_window=(15, 55)),
        DeliveryPoint(id=18, pos=(50, 85), weight=3.0, priority=1, time_window=(60, 100)),
        DeliveryPoint(id=19, pos=(80, 70), weight=2.5, priority=4, time_window=(20, 60)),
        DeliveryPoint(id=20, pos=(30, 55), weight=1.5, priority=2, time_window=(40, 80)),
    ]
    scenario1_no_fly_zones = [
        NoFlyZone(id=1, coordinates=[(40, 30), (60, 30), (60, 50), (40, 50)], active_time=(0, 120)),
        NoFlyZone(id=2, coordinates=[(70, 10), (90, 10), (90, 30), (70, 30)], active_time=(30, 90)),
        NoFlyZone(id=3, coordinates=[(10, 60), (30, 60), (30, 80), (10, 80)], active_time=(0, 60))
    ]
    scenario1_data = {
        'drones': scenario1_drones,
        'deliveries': scenario1_deliveries,
        'no_fly_zones': scenario1_no_fly_zones
    }
    
    # Senaryo 1: 5 drone, 20 teslimat, 2 no-fly zone (sabit veri ile)
    scenario1_result = run_scenario("Senaryo 1", 5, 20, 3, use_fixed_data=True, fixed_data=scenario1_data)
    
    # Senaryo 2: 10 drone, 50 teslimat, 5 dinamik no-fly zone (rastgele veri ile)
    scenario2_result = run_scenario("Senaryo 2", 10, 50, 5)
    
    # Görselleştirme
    print_separator("GÖRSELLEŞTİRME")
    
    print("Senaryo 1 görselleştiriliyor...")
    plot_routes(scenario1_result['drones'], scenario1_result['deliveries'], 
                scenario1_result['no_fly_zones'], scenario1_result['ga_routes'], 
                scenario1_result['graph'], filename="output/scenario1_routes.png")
    
    print("Senaryo 2 görselleştiriliyor...")
    plot_routes(scenario2_result['drones'], scenario2_result['deliveries'], 
                scenario2_result['no_fly_zones'], scenario2_result['ga_routes'], 
                scenario2_result['graph'], filename="output/scenario2_routes.png")
    
    print("Tüm analizler tamamlandı!")

if __name__ == "__main__":
    main()