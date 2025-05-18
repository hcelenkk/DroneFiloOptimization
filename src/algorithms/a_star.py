import heapq
from typing import List, Tuple, Dict, Optional
from src.utils.graph import Graph
from src.models.drone import Drone
from src.models.delivery_point import DeliveryPoint

class AStar:
    def __init__(self, graph: Graph):
        self.graph = graph

    def _heuristic(self, node: str, goal: str, delivery_points: List[DeliveryPoint]) -> float:
        """Hedef düğüme olan tahmini mesafe (Öklid mesafesi) + uçuş yasağı cezası."""
        # Düğüm ID'sinden koordinatları al
        node_pos = self._get_node_position(node)
        goal_pos = self._get_node_position(goal)
        
        # Öklid mesafesi
        distance = ((node_pos[0] - goal_pos[0]) ** 2 + (node_pos[1] - goal_pos[1]) ** 2) ** 0.5
        
        # Uçuş yasağı cezası (basit bir sabit ceza, gerekirse geliştirilebilir)
        nofly_penalty = 0
        for nfz in self.graph.no_fly_zones:
            for i in range(len(nfz.coordinates)):
                p1 = nfz.coordinates[i]
                p2 = nfz.coordinates[(i + 1) % len(nfz.coordinates)]
                if self.graph._lines_intersect(node_pos, goal_pos, p1, p2):
                    nofly_penalty += 1000  # Büyük bir ceza ekle
        
        return distance + nofly_penalty

    def _get_node_position(self, node: str) -> Tuple[float, float]:
        """Düğüm ID'sinden koordinatları döndürür."""
        if node.startswith("drone_"):
            drone_id = int(node.split("_")[1])
            for drone in self.graph.drones:
                if drone.id == drone_id:
                    return drone.start_pos
        elif node.startswith("dp_"):
            dp_id = int(node.split("_")[1])
            for dp in self.graph.delivery_points:
                if dp.id == dp_id:
                    return dp.pos
        raise ValueError(f"Düğüm {node} bulunamadı.")

    def find_path(self, start: str, goal: str, drone: Drone) -> Tuple[Optional[List[str]], float]:
        """A* algoritması ile en kısa yolu bulur."""
        # Min-heap için öncelik kuyruğu: (f_score, node, path, cost, current_weight, battery_used)
        open_set = [(0, start, [start], 0.0, 0.0, 0)]
        heapq.heapify(open_set)
        
        # Ziyaret edilen düğümler
        closed_set = set()
        
        # g_score: Başlangıçtan düğüme gerçek maliyet
        g_score = {start: 0}
        # f_score: g_score + heuristic
        f_score = {start: self._heuristic(start, goal, self.graph.delivery_points)}
        
        while open_set:
            _, current, path, current_cost, current_weight, battery_used = heapq.heappop(open_set)
            
            if current == goal:
                return path, current_cost
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            # Komşuları tara
            for neighbor, distance, cost in self.graph.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                # Drone kapasite kontrolü
                neighbor_weight = 0.0
                if neighbor.startswith("dp_"):
                    dp_id = int(neighbor.split("_")[1])
                    for dp in self.graph.delivery_points:
                        if dp.id == dp_id:
                            neighbor_weight = dp.weight
                            break
                
                if current_weight + neighbor_weight > drone.max_weight:
                    continue  # Ağırlık sınırı aşıldı
                
                # Batarya tüketimi (basit model: mesafe * hız başına tüketim)
                battery_consumption = distance * 10  # 10 mAh/metre (örnek sabit)
                if battery_used + battery_consumption > drone.battery:
                    continue  # Batarya yetersiz
                
                # Yeni maliyet
                tentative_g_score = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self._heuristic(neighbor, goal, self.graph.delivery_points)
                    
                    # Yeni yolu ekle
                    new_path = path + [neighbor]
                    heapq.heappush(open_set, (
                        f_score[neighbor],
                        neighbor,
                        new_path,
                        tentative_g_score,
                        current_weight + neighbor_weight,
                        battery_used + battery_consumption
                    ))
        
        return None, float('inf')  # Yol bulunamadı