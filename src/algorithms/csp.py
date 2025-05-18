from typing import List, Dict, Optional
from src.models.drone import Drone
from src.models.delivery_point import DeliveryPoint
from src.utils.graph import Graph

class CSP:
    def __init__(self, drones: List[Drone], delivery_points: List[DeliveryPoint], graph: Graph):
        self.drones = drones
        self.delivery_points = delivery_points
        self.graph = graph
        self.assignments = {}  # Drone ID -> Teslimat noktası ID

    def _is_valid_assignment(self, drone: Drone, dp: DeliveryPoint, current_time: str) -> bool:
        """Bir teslimatın drone'a atanmasının geçerli olup olmadığını kontrol eder."""
        # Ağırlık kontrolü
        if dp.weight > drone.max_weight:
            return False
        
        # Batarya kontrolü (basit model: mesafe * 10 mAh/metre)
        start_pos = drone.start_pos
        dp_pos = dp.pos
        distance = ((start_pos[0] - dp_pos[0]) ** 2 + (start_pos[1] - dp_pos[1]) ** 2) ** 0.5
        battery_consumption = distance * 10
        if battery_consumption > drone.battery:
            return False
        
        # Zaman penceresi kontrolü (basit: teslimat zaman aralığında mı?)
        dp_start, dp_end = dp.time_window
        if not (dp_start <= current_time <= dp_end):
            return False
        
        # Aynı anda tek paket kontrolü
        for assigned_dp_id in self.assignments.values():
            if assigned_dp_id == dp.id:
                return False
        
        return True

    def _backtrack(self, drone_index: int, current_time: str) -> Optional[Dict[int, int]]:
        """Geri izleme ile geçerli atamaları bulur."""
        if drone_index >= len(self.drones):
            return self.assignments
        
        drone = self.drones[drone_index]
        
        for dp in self.delivery_points:
            if self._is_valid_assignment(drone, dp, current_time):
                self.assignments[drone.id] = dp.id
                result = self._backtrack(drone_index + 1, current_time)
                if result is not None:
                    return result
                del self.assignments[drone.id]
        
        # Eğer bu drone'a atama yapılamazsa, bir sonraki drone'a geç
        return self._backtrack(drone_index + 1, current_time)

    def solve(self, current_time: str = "10:00") -> Optional[Dict[int, int]]:
        """CSP ile teslimat atamalarını çözer."""
        self.assignments = {}
        return self._backtrack(0, current_time)