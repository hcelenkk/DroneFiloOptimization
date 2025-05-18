import math
from typing import List, Tuple, Dict
from src.models.delivery_point import DeliveryPoint
from src.models.no_fly_zone import NoFlyZone
from src.models.drone import Drone

class Graph:
    def __init__(self, drones: List[Drone], delivery_points: List[DeliveryPoint], no_fly_zones: List[NoFlyZone]):
        self.drones = drones
        self.delivery_points = delivery_points
        self.no_fly_zones = no_fly_zones
        self.graph = self._build_graph()

    def _calculate_distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """İki nokta arasındaki Öklid mesafesini hesaplar."""
        x1, y1 = pos1
        x2, y2 = pos2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def _is_line_intersecting_no_fly_zone(self, start: Tuple[float, float], end: Tuple[float, float]) -> bool:
        """Bir çizginin uçuş yasağı bölgesini kesip kesmediğini kontrol eder."""
        for nfz in self.no_fly_zones:
            for i in range(len(nfz.coordinates)):
                p1 = nfz.coordinates[i]
                p2 = nfz.coordinates[(i + 1) % len(nfz.coordinates)]
                if self._lines_intersect(start, end, p1, p2):
                    return True
        return False

    def _lines_intersect(self, a1: Tuple[float, float], a2: Tuple[float, float], 
                        b1: Tuple[float, float], b2: Tuple[float, float]) -> bool:
        """İki çizginin kesişip kesişmediğini kontrol eder."""
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        return ccw(a1, b1, b2) != ccw(a2, b1, b2) and ccw(a1, a2, b1) != ccw(a1, a2, b2)

    def _build_graph(self) -> Dict[str, List[Tuple[str, float, float]]]:
        """Teslimat noktaları ve drone başlangıç pozisyonları için grafik oluşturur."""
        graph = {}
        nodes = []

        # Drone başlangıç pozisyonlarını düğüm olarak ekle
        for drone in self.drones:
            node_id = f"drone_{drone.id}"
            nodes.append((node_id, drone.start_pos, 0.0, 0))  # weight=0, priority=0

        # Teslimat noktalarını düğüm olarak ekle
        for dp in self.delivery_points:
            node_id = f"dp_{dp.id}"
            nodes.append((node_id, dp.pos, dp.weight, dp.priority))

        # Her düğüm için komşuluk listesi oluştur
        for i, (node_id, pos, _, _) in enumerate(nodes):
            graph[node_id] = []
            for j, (other_node_id, other_pos, other_weight, other_priority) in enumerate(nodes):
                if i != j:  # Kendine kenar ekleme
                    distance = self._calculate_distance(pos, other_pos)
                    # Uçuş yasağı bölgesini kontrol et
                    if self._is_line_intersecting_no_fly_zone(pos, other_pos):
                        continue  # Uçuş yasağı bölgesini kesiyorsa kenar ekleme
                    # Maliyet fonksiyonu: distance * weight + (priority * 100)
                    # Drone'dan teslimat noktasına giderken hedefin ağırlığı ve önceliği kullanılır
                    cost = distance * other_weight + (other_priority * 100)
                    graph[node_id].append((other_node_id, distance, cost))

        return graph

    def get_neighbors(self, node: str) -> List[Tuple[str, float, float]]:
        """Bir düğümün komşularını ve kenar maliyetlerini döndürür."""
        return self.graph.get(node, [])