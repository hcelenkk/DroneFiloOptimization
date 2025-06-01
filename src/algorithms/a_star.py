from heapq import heappush, heappop
from typing import List, Tuple, Dict
from src.utils.graph import Graph
from src.models.drone import Drone

class AStar:
    def __init__(self, graph: Graph):
        self.graph = graph

    def _heuristic(self, node1: str, node2: str, drone: Drone, current_time: int = 0) -> float:
        """Heuristic fonksiyonu: mesafe + dinamik uçuş yasağı cezası."""
        pos1 = self.graph.get_node_position(node1)
        pos2 = self.graph.get_node_position(node2)
        distance = ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5
        
        # Dinamik no-fly zone kontrolü
        penalty = 0
        for nfz in self.graph.no_fly_zones:
            if nfz.is_active(current_time) and self.graph._is_line_intersecting_no_fly_zone(pos1, pos2, nfz.coordinates):
                penalty = 1000
                break
        
        return distance + penalty

    def _cost(self, node1: str, node2: str, drone: Drone) -> float:
        """Maliyet fonksiyonu: Cost(distance) = distance × weight + (priority × 100)"""
        pos1 = self.graph.get_node_position(node1)
        pos2 = self.graph.get_node_position(node2)
        distance = ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5
    
        if node2.startswith("dp_"):
            try:
                dp_id = int(node2.split("_")[1])
                dp = next(dp for dp in self.graph.delivery_points if dp.id == dp_id)
                
                # Kapasite kontrolü
                if dp.weight > drone.max_weight:
                    return float('inf')
                
                # Priority mantığı düzeltildi: Yüksek öncelik = düşük maliyet
                priority_bonus = (6 - dp.priority) * 100  # Priority 5→100, Priority 1→500
                cost = distance * dp.weight + priority_bonus
                return cost
            except StopIteration:
                return distance * 1.0
        return distance
 
    def find_path(self, start: str, goal: str, drone: Drone, current_time: int = 0) -> Tuple[List[str], float]:
        """A* algoritması ile en kısa yolu bulur."""
        open_set = []
        heappush(open_set, (0, start))
        came_from: Dict[str, str] = {}
        g_score: Dict[str, float] = {start: 0}
        f_score: Dict[str, float] = {start: self._heuristic(start, goal, drone, current_time)}

        while open_set:
            current = heappop(open_set)[1]
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1], g_score[goal]

            for neighbor in self.graph.get_neighbors(current):
                tentative_g_score = g_score[current] + self._cost(current, neighbor, drone)
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self._heuristic(neighbor, goal, drone, current_time)
                    if neighbor not in [item[1] for item in open_set]:
                        heappush(open_set, (f_score[neighbor], neighbor))

        return [], float('inf')