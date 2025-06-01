from typing import List, Tuple
import random
from src.utils.graph import Graph
from src.models.drone import Drone
from src.models.delivery_point import DeliveryPoint

class GeneticAlgorithm:
    def __init__(self, drones: List[Drone], delivery_points: List[DeliveryPoint], graph: Graph):
        self.drones = drones
        self.delivery_points = delivery_points
        self.graph = graph
        self.population_size = 200
        self.generations = 100
        self.valid_dp_ids = [dp.id for dp in self.delivery_points]

    def validate_chromosome(self, chromosome: List[List[int]]) -> Tuple[bool, str]:
        """Chromosome'da duplicate teslimat ve tek paket kısıtını kontrol et"""
        all_deliveries = []
        
        # Her drone'un rotasını kontrol et
        for i, drone_route in enumerate(chromosome):
            # TEK PAKET KISITI: Her drone maksimum 1 teslimat yapabilir
            if len(drone_route) > 1:
                return False, f"Drone {i} has {len(drone_route)} deliveries, but can only carry 1 package"
            
            all_deliveries.extend(drone_route)
        
        # Duplicate kontrol
        unique_deliveries = set(all_deliveries)
        if len(all_deliveries) != len(unique_deliveries):
            duplicates = [x for x in all_deliveries if all_deliveries.count(x) > 1]
            return False, f"Duplicate deliveries found: {set(duplicates)}"
        
        # Geçersiz teslimat ID kontrol
        valid_delivery_ids = set(self.valid_dp_ids)
        invalid_ids = unique_deliveries - valid_delivery_ids
        if invalid_ids:
            return False, f"Invalid delivery IDs: {invalid_ids}"
            
        return True, "Valid"

    def repair_chromosome(self, chromosome: List[List[int]]) -> List[List[int]]:
        """Bozuk chromosome'u onar - tek paket kısıtını uygula"""
        # Her drone'dan sadece ilk teslimatı al (tek paket kısıtı)
        repaired_chromosome = []
        used_deliveries = set()
        
        for drone_route in chromosome:
            if drone_route and drone_route[0] not in used_deliveries:
                # İlk teslimatı al ve kullanıldı olarak işaretle
                repaired_chromosome.append([drone_route[0]])
                used_deliveries.add(drone_route[0])
            else:
                # Boş route veya zaten kullanılmış teslimat
                repaired_chromosome.append([])
        
        # Atanmamış teslimatları boş drone'lara dağıt
        all_delivery_ids = set(self.valid_dp_ids)
        unassigned = list(all_delivery_ids - used_deliveries)
        
        for delivery_id in unassigned:
            # Boş drone bul
            empty_drone_idx = None
            for i, route in enumerate(repaired_chromosome):
                if not route:  # Boş drone
                    empty_drone_idx = i
                    break
            
            if empty_drone_idx is not None:
                # Drone kapasitesini kontrol et
                delivery = next(dp for dp in self.delivery_points if dp.id == delivery_id)
                drone = self.drones[empty_drone_idx]
                
                if delivery.weight <= drone.max_weight:
                    repaired_chromosome[empty_drone_idx] = [delivery_id]
            
        return repaired_chromosome

    def find_best_drone_for_delivery(self, delivery_id: int, current_chromosome: List[List[int]]) -> int:
        """Bir teslimat için en uygun BOŞ drone'u bul"""
        try:
            delivery = next(dp for dp in self.delivery_points if dp.id == delivery_id)
        except StopIteration:
            return None
        
        best_drone_idx = None
        best_score = float('inf')
        
        for i, drone in enumerate(self.drones):
            # Sadece boş drone'ları değerlendir (tek paket kısıtı)
            if current_chromosome[i]:  # Drone zaten dolu
                continue
            
            # Kapasite kontrolü
            if delivery.weight > drone.max_weight:
                continue
                
            # Mesafe hesabı
            distance = ((drone.start_pos[0] - delivery.pos[0])**2 + 
                       (drone.start_pos[1] - delivery.pos[1])**2)**0.5
            
            if distance < best_score:
                best_score = distance
                best_drone_idx = i
                
        return best_drone_idx

    def _fitness(self, routes: List[List[int]]) -> float:
        """
        Fitness fonksiyonu: Tek paket kısıtını da kontrol eder
        """
        # Önce validasyon kontrol
        is_valid, error_msg = self.validate_chromosome(routes)
        if not is_valid:
            return float('-inf')
        
        total_deliveries = sum(len(route) for route in routes if route)
        total_energy = 0
        violations = 0
        
        for i, route in enumerate(routes):
            if not route:
                continue
            
            # Tek paket kısıtı kontrolü (ekstra güvenlik)
            if len(route) > 1:
                violations += 10  # Çok ağır penaltı
                continue
                
            current_pos = self.drones[i].start_pos
            drone = self.drones[i]
            dp_id = route[0]  # Sadece bir teslimat var
            
            try:
                dp = next(dp for dp in self.delivery_points if dp.id == dp_id)
                
                # Mesafe hesaplama
                distance = ((current_pos[0] - dp.pos[0]) ** 2 + (current_pos[1] - dp.pos[1]) ** 2) ** 0.5
                
                # Enerji tüketimi hesaplama
                energy_consumption = distance * 5 / drone.speed
                total_energy += energy_consumption
                
                # Kapasite ihlali kontrolü
                if dp.weight > drone.max_weight:
                    violations += 1
                
                # No-fly zone ihlali kontrolü
                if self.graph.is_in_no_fly_zone(current_pos, dp.pos):
                    violations += 1
                
            except StopIteration:
                violations += 1
                continue
        
        # Fitness formülü
        fitness = (total_deliveries * 50) - (total_energy * 0.1) - (violations * 1000)
        return fitness

    def _crossover(self, parent1: List[int], parent2: List[int]) -> List[int]:
        """Çaprazlama işlemi - Tek paket için basitleştirildi"""
        # Tek paket taşıyabildiği için crossover daha basit
        if not parent1 and not parent2:
            return []
        if not parent1:
            return parent2.copy() if len(parent2) <= 1 else [parent2[0]]
        if not parent2:
            return parent1.copy() if len(parent1) <= 1 else [parent1[0]]
        
        # Rastgele birini seç (tek paket kısıtı)
        if random.random() < 0.5:
            return [parent1[0]] if parent1 else []
        else:
            return [parent2[0]] if parent2 else []

    def _mutate(self, route: List[int]) -> List[int]:
        """Mutasyon işlemi - Tek paket için"""
        if random.random() < 0.1:  # %10 mutasyon şansı
            if route:
                # Mevcut teslimatı rastgele başka bir teslimatla değiştir
                available_dps = [dp_id for dp_id in self.valid_dp_ids if dp_id != route[0]]
                if available_dps and random.random() < 0.3:
                    route[0] = random.choice(available_dps)
            else:
                # Boş rotaya rastgele teslimat ekle
                if random.random() < 0.2:
                    route.append(random.choice(self.valid_dp_ids))
        
        # Tek paket kısıtını zorla
        if len(route) > 1:
            route = [route[0]]
            
        return route

    def _generate_initial_population(self) -> List[List[List[int]]]:
        """Başlangıç popülasyonunu üret - Tek paket kısıtı ile"""
        population = []
        
        for _ in range(self.population_size):
            individual = []
            used_dps = set()
            
            for drone in self.drones:
                # Her drone için maksimum 1 teslimat
                available_dps = [dp_id for dp_id in self.valid_dp_ids if dp_id not in used_dps]
                
                if available_dps:
                    # Drone kapasitesine uygun DP'leri filtrele
                    suitable_dps = []
                    for dp_id in available_dps:
                        dp = next(dp for dp in self.delivery_points if dp.id == dp_id)
                        if dp.weight <= drone.max_weight:
                            suitable_dps.append(dp_id)
                    
                    if suitable_dps and random.random() < 0.7:  # %70 şansla teslimat ata
                        selected_dp = random.choice(suitable_dps)
                        route = [selected_dp]
                        used_dps.add(selected_dp)
                    else:
                        route = []
                else:
                    route = []
                    
                individual.append(route)
            
            population.append(individual)
        
        return population

    def run(self, current_time: str = "00:00") -> Tuple[List[List[int]], float]:
        """Genetik algoritmayı çalıştırır - Tek paket kısıtı ile"""
        population = self._generate_initial_population()
        
        for generation in range(self.generations):
            # Fitness değerlerine göre sırala
            population = sorted(population, key=self._fitness, reverse=True)
            
            # En iyi %25'i koru (elitism)
            elite_size = self.population_size // 4
            new_population = population[:elite_size]
            
            # Kalan popülasyonu üret
            while len(new_population) < self.population_size:
                # Tournament selection
                parent1 = self._tournament_selection(population[:self.population_size//2])
                parent2 = self._tournament_selection(population[:self.population_size//2])
                
                # Crossover ve mutation
                child = []
                for p1_route, p2_route in zip(parent1, parent2):
                    child_route = self._crossover(p1_route, p2_route)
                    child_route = self._mutate(child_route)
                    child.append(child_route)
                
                # Çocuğu kontrol et ve onar
                is_valid, error_msg = self.validate_chromosome(child)
                if not is_valid:
                    child = self.repair_chromosome(child)
                
                new_population.append(child)
            
            population = new_population
            
            # Progress log
            if generation % 20 == 0:
                best_individual = sorted(population, key=self._fitness, reverse=True)[0]
                delivered_count = sum(len(route) for route in best_individual)
                print(f"Generation {generation}: {delivered_count}/{len(self.delivery_points)} teslimat yapıldı.")
        
        # En iyi çözümü döndür
        best_individual = sorted(population, key=self._fitness, reverse=True)[0]
        best_fitness = self._fitness(best_individual)
        
        return best_individual, best_fitness

    def _tournament_selection(self, population: List[List[List[int]]], tournament_size: int = 3) -> List[List[int]]:
        """Tournament selection ile parent seçimi."""
        tournament = random.sample(population, min(tournament_size, len(population)))
        return max(tournament, key=self._fitness)