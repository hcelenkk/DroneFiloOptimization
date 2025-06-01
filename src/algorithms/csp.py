from typing import List, Dict, Tuple
from src.models.drone import Drone
from src.models.delivery_point import DeliveryPoint, sort_deliveries_by_priority
from src.models.no_fly_zone import NoFlyZone

class CSP:
    def __init__(self, drones: List[Drone], delivery_points: List[DeliveryPoint], no_fly_zones: List[NoFlyZone]):
        self.drones = drones
        self.delivery_points = sort_deliveries_by_priority(delivery_points)  # Teslimatları öncelik sırasına göre sırala
        self.no_fly_zones = no_fly_zones
        self.assignments = {}

    def _check_no_fly_zone_violation(self, pos1: Tuple[float, float], pos2: Tuple[float, float], current_time: int) -> bool:
        """İki nokta arasındaki çizginin uçuş yasağı bölgesini ihlal edip etmediğini kontrol eder."""
        for no_fly_zone in self.no_fly_zones:
            if no_fly_zone.is_active(current_time):
                # Basit kesişim kontrolü - daha karmaşık geometrik hesaplama gerekebilir
                if self._line_intersects_polygon(pos1, pos2, no_fly_zone.coordinates):
                    return True
        return False

    def _line_intersects_polygon(self, pos1: Tuple[float, float], pos2: Tuple[float, float], polygon: List[Tuple[float, float]]) -> bool:
        """Çizgi parçasının çokgen ile kesişip kesişmediğini kontrol eder."""
        x1, y1 = pos1
        x2, y2 = pos2
        
        for i in range(len(polygon)):
            x3, y3 = polygon[i]
            x4, y4 = polygon[(i + 1) % len(polygon)]
            
            # Basit bounding box kontrolü
            if (max(x1, x2) >= min(x3, x4) and max(x3, x4) >= min(x1, x2) and
                max(y1, y2) >= min(y3, y4) and max(y3, y4) >= min(y1, y2)):
                return True
        return False

    def _is_valid_assignment(self, drone: Drone, dp: DeliveryPoint, current_time_minutes: int, 
                           current_pos: Tuple[float, float], remaining_battery: float) -> bool:
        """Atamanın geçerli olup olmadığını kontrol eder."""
        distance = ((current_pos[0] - dp.pos[0]) ** 2 + (current_pos[1] - dp.pos[1]) ** 2) ** 0.5
        
        # Ağırlık kontrolü - Drone kapasitesini aşan rotaları eleyin
        if dp.weight > drone.max_weight:
            return False
        
        # Batarya kontrolü
        battery_consumption = distance * (5 / drone.speed)
        if battery_consumption > remaining_battery:
            return False
        
        # Şarj kontrolü
        if remaining_battery - battery_consumption < 20:
            charge_time = drone.charge()  # Şarj süresi saniye cinsinden
            current_time_minutes += charge_time / 60  # Dakikaya çevir

        # Zaman penceresi kontrolü
        travel_time = distance / drone.speed * 60  # Dakika cinsinden
        delivery_time = current_time_minutes + travel_time
        dp_start, dp_end = dp.time_window
        if not (dp_start <= delivery_time <= dp_end):
            return False
        
        # Uçuş yasağı bölgesi kontrolü
        if self._check_no_fly_zone_violation(current_pos, dp.pos, current_time_minutes):
            return False
        
        return True

    def _calculate_assignment_cost(self, drone: Drone, dp: DeliveryPoint, distance: float) -> float:
        """
        Atama maliyetini hesaplar.
        Cost(distance) = distance × weight + (priority × 100)
        Priority mantığı düzeltildi: Yüksek öncelik = düşük maliyet
        """
        priority_bonus = (6 - dp.priority) * 100  # Priority 5→100, Priority 1→500
        return distance * dp.weight + priority_bonus

    def _backtrack(self, assignment: Dict[int, List[int]], unassigned_dps: List[DeliveryPoint], 
                  drone_states: Dict[int, Dict]) -> Dict[int, List[int]]:
        """
        Geliştirilmiş geri izleme algoritması.
        drone_states: {drone_id: {'pos': (x,y), 'battery': float, 'time': int}}
        """
        if not unassigned_dps:
            return assignment.copy()
        
        # En yüksek öncelikli teslimat noktasını seç
        dp = unassigned_dps[0]
        
        # Her drone için bu DP'yi atamayı dene
        best_assignment = None
        best_cost = float('inf')
        
        for drone in self.drones:
            drone_id = drone.id
            current_state = drone_states[drone_id]
            
            if self._is_valid_assignment(drone, dp, current_state['time'], 
                                       current_state['pos'], current_state['battery']):
                
                # Maliyeti hesapla
                distance = ((current_state['pos'][0] - dp.pos[0]) ** 2 + 
                           (current_state['pos'][1] - dp.pos[1]) ** 2) ** 0.5
                cost = self._calculate_assignment_cost(drone, dp, distance)
                
                if cost < best_cost:
                    # Bu atamayı dene
                    new_assignment = assignment.copy()
                    if drone_id not in new_assignment:
                        new_assignment[drone_id] = []
                    new_assignment[drone_id].append(dp.id)
                    
                    # Drone durumunu güncelle
                    new_drone_states = drone_states.copy()
                    battery_consumption = distance * (5 / drone.speed)
                    travel_time = distance / drone.speed * 60
                    
                    new_drone_states[drone_id] = {
                        'pos': dp.pos,
                        'battery': current_state['battery'] - battery_consumption,
                        'time': current_state['time'] + travel_time
                    }
                    
                    # Şarj gerekli mi?
                    if new_drone_states[drone_id]['battery'] < 20:
                        charge_time = drone.charge()
                        new_drone_states[drone_id]['battery'] = 100
                        new_drone_states[drone_id]['time'] += charge_time / 60
                    
                    # Recursive call
                    result = self._backtrack(new_assignment, unassigned_dps[1:], new_drone_states)
                    if result is not None:
                        best_assignment = result
                        best_cost = cost
        
        return best_assignment

    def solve(self, current_time: str = "00:00") -> Dict[int, List[int]]:
        """CSP problemini çözerek her drone'a teslimat noktaları atar."""
        # Başlangıç durumunu ayarla
        current_time_minutes = int(current_time.split(':')[0]) * 60 + int(current_time.split(':')[1])
        
        # Drone durumlarını başlat
        drone_states = {}
        for drone in self.drones:
            drone_states[drone.id] = {
                'pos': drone.start_pos,
                'battery': drone.battery,
                'time': current_time_minutes
            }
        
        # Tüm teslimat noktalarını başlangıçta atanmamış olarak işaretle
        unassigned_dps = self.delivery_points.copy()
        
        # Backtracking ile çözüm bul
        result = self._backtrack({}, unassigned_dps, drone_states)
        
        # Eğer çözüm bulunamazsa, greedy yaklaşım kullan
        if result is None:
            result = self._greedy_fallback(current_time_minutes)
        
        return result if result is not None else {}

    def _greedy_fallback(self, current_time_minutes: int) -> Dict[int, List[int]]:
        """Backtracking başarısız olursa greedy yaklaşım kullan."""
        assignments = {drone.id: [] for drone in self.drones}
        drone_states = {}
        
        # Drone durumlarını başlat
        for drone in self.drones:
            drone_states[drone.id] = {
                'pos': drone.start_pos,
                'battery': drone.battery,
                'time': current_time_minutes
            }
        
        # Her teslimat noktası için en uygun drone'u bul
        for dp in self.delivery_points:
            best_drone = None
            best_cost = float('inf')
            
            for drone in self.drones:
                drone_id = drone.id
                current_state = drone_states[drone_id]
                
                if self._is_valid_assignment(drone, dp, current_state['time'], 
                                           current_state['pos'], current_state['battery']):
                    
                    distance = ((current_state['pos'][0] - dp.pos[0]) ** 2 + 
                               (current_state['pos'][1] - dp.pos[1]) ** 2) ** 0.5
                    cost = self._calculate_assignment_cost(drone, dp, distance)
                    
                    if cost < best_cost:
                        best_cost = cost
                        best_drone = drone
            
            # En iyi drone'a ata
            if best_drone is not None:
                drone_id = best_drone.id
                assignments[drone_id].append(dp.id)
                
                # Drone durumunu güncelle
                distance = ((drone_states[drone_id]['pos'][0] - dp.pos[0]) ** 2 + 
                           (drone_states[drone_id]['pos'][1] - dp.pos[1]) ** 2) ** 0.5
                battery_consumption = distance * (5 / best_drone.speed)
                travel_time = distance / best_drone.speed * 60
                
                drone_states[drone_id]['pos'] = dp.pos
                drone_states[drone_id]['battery'] -= battery_consumption
                drone_states[drone_id]['time'] += travel_time
                
                # Şarj kontrolü
                if drone_states[drone_id]['battery'] < 20:
                    charge_time = best_drone.charge()
                    drone_states[drone_id]['battery'] = 100
                    drone_states[drone_id]['time'] += charge_time / 60
        
        return assignments