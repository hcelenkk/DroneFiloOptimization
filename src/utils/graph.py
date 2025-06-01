from typing import List, Tuple, Dict
from src.models.drone import Drone
from src.models.delivery_point import DeliveryPoint
from src.models.no_fly_zone import NoFlyZone

class Graph:
    def __init__(self, drones: List[Drone], delivery_points: List[DeliveryPoint], no_fly_zones: List[NoFlyZone]):
        self.drones = drones
        self.delivery_points = delivery_points  # Hata ayıklaması için kontrol
        self.no_fly_zones = no_fly_zones
        self.edges = {}
        #print(f"Graph initialized with {len(delivery_points)} delivery points: {[dp.id for dp in delivery_points]}")  # Hata ayıklaması
        self._build_graph()

    def _build_graph(self):
        """Grafı oluştur: düğümler (dronelar ve teslimat noktaları) ve kenarları (mesafeler)."""
        nodes = [f"drone_{drone.id}" for drone in self.drones] + [f"dp_{dp.id}" for dp in self.delivery_points]
        #print(f"Generated nodes: {nodes}")  # Hata ayıklaması
        for node1 in nodes:
            self.edges[node1] = {}
            for node2 in nodes:
                if node1 != node2:
                    pos1 = self.get_node_position(node1)
                    pos2 = self.get_node_position(node2)
                    distance = ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5
                    self.edges[node1][node2] = distance

    def get_node_position(self, node: str) -> Tuple[float, float]:
        """Verilen düğümün (drone_X veya dp_Y) koordinatlarını döndür."""
        if node.startswith("drone_"):
            drone_id = int(node.split("_")[1])
            drone = next(d for d in self.drones if d.id == drone_id)
            return drone.start_pos
        elif node.startswith("dp_"):
            dp_id = int(node.split("_")[1])
            dp = next(dp for dp in self.delivery_points if dp.id == dp_id)
            return dp.pos
        raise ValueError(f"Geçersiz düğüm: {node}")

    def get_neighbors(self, node: str) -> List[str]:
        """Verilen düğümün komşularını döndür."""
        return list(self.edges[node].keys())

    def _is_line_intersecting_no_fly_zone(self, pos1: Tuple[float, float], pos2: Tuple[float, float], coordinates: List[Tuple[float, float]]) -> bool:
        """İki nokta arasındaki çizginin uçuş yasağı bölgesini kesip kesmediğini kontrol eder."""
        x1, y1 = pos1
        x2, y2 = pos2
        for i in range(len(coordinates)):
            x3, y3 = coordinates[i]
            x4, y4 = coordinates[(i + 1) % len(coordinates)]
            if (max(x1, x2) >= min(x3, x4) and max(x3, x4) >= min(x1, x2) and
                max(y1, y2) >= min(y3, y4) and max(y3, y4) >= min(y1, y2)):
                return True
        return False

    def is_in_no_fly_zone(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> bool:
        """İki nokta arasındaki yolun uçuş yasağı bölgesine girip girmediğini kontrol eder."""
        for no_fly_zone in self.no_fly_zones:
            if self._is_line_intersecting_no_fly_zone(pos1, pos2, no_fly_zone.coordinates):
                return True
        return False