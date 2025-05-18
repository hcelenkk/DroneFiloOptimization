import random
from typing import List, Tuple, Dict
from src.models.drone import Drone
from src.models.delivery_point import DeliveryPoint
from src.utils.graph import Graph

class GeneticAlgorithm:
    def __init__(self, drones: List[Drone], delivery_points: List[DeliveryPoint], graph: Graph,
                 population_size: int = 100, generations: int = 100, mutation_rate: float = 0.1):
        self.drones = drones
        self.delivery_points = delivery_points
        self.graph = graph
        self.population_size = population_size
        self.generations = generations
        self.mutation_rate = mutation_rate
        self.population = self._initialize_population()

    def _initialize_population(self) -> List[List[List[int]]]:
        """Rastgele popülasyon oluşturur: her birey, drone'lara atanmış teslimat sıralarını içerir."""
        population = []
        dp_ids = [dp.id for dp in self.delivery_points]
        
        for _ in range(self.population_size):
            individual = []
            remaining_dps = dp_ids.copy()
            for _ in self.drones:
                # Her drone için rastgele bir teslimat sırası
                if not remaining_dps:
                    individual.append([])
                    continue
                route_length = random.randint(0, len(remaining_dps))
                route = random.sample(remaining_dps, route_length)
                individual.append(route)
                # Atanan teslimatları kalan listeden çıkar
                remaining_dps = [dp for dp in remaining_dps if dp not in route]
            population.append(individual)
        
        return population

    def _calculate_fitness(self, individual: List[List[int]], current_time: str) -> float:
        """Fitness fonksiyonu: teslimat sayısı, enerji tüketimi ve kısıt ihlallerine göre."""
        total_deliveries = 0
        total_energy = 0.0
        constraint_violations = 0

        # Her drone'un rotasını değerlendir
        for drone_idx, route in enumerate(individual):
            drone = self.drones[drone_idx]
            current_weight = 0.0
            current_pos = drone.start_pos
            battery_used = 0.0

            for dp_id in route:
                # Teslimat noktasını bul
                dp = next((dp for dp in self.delivery_points if dp.id == dp_id), None)
                if not dp:
                    constraint_violations += 1
                    continue

                # Kısıt kontrolleri
                # 1. Ağırlık
                if current_weight + dp.weight > drone.max_weight:
                    constraint_violations += 1
                    continue
                
                # 2. Batarya (mesafe * 10 mAh/metre)
                distance = ((current_pos[0] - dp.pos[0]) ** 2 + (current_pos[1] - dp.pos[1]) ** 2) ** 0.5
                battery_consumption = distance * 10
                if battery_used + battery_consumption > drone.battery:
                    constraint_violations += 1
                    continue
                
                # 3. Zaman penceresi
                dp_start, dp_end = dp.time_window
                if not (dp_start <= current_time <= dp_end):
                    constraint_violations += 1
                    continue

                # Geçerli teslimat
                total_deliveries += 1
                current_weight += dp.weight
                battery_used += battery_consumption
                total_energy += battery_consumption
                current_pos = dp.pos

        # Fitness: (teslimat sayısı * 50) - (enerji * 0.1) - (ihlaller * 1000)
        return (total_deliveries * 50) - (total_energy * 0.1) - (constraint_violations * 1000)

    def _tournament_selection(self, population: List[List[List[int]]], tournament_size: int = 5) -> List[List[int]]:
        """Turnuva seçimi ile bir birey seçer."""
        tournament = random.sample(population, tournament_size)
        return max(tournament, key=lambda ind: self._calculate_fitness(ind, "10:00"))

    def _crossover(self, parent1: List[List[int]], parent2: List[List[int]]) -> Tuple[List[List[int]], List[List[int]]]:
        """Sıralı çaprazlama (ordered crossover) ile iki çocuk üretir."""
        child1, child2 = [], []
        
        for route1, route2 in zip(parent1, parent2):
            if len(route1) < 2 or len(route2) < 2:
                # Rota 2'den küçükse, çaprazlama yapmadan kopyala
                child1.append(route1[:])
                child2.append(route2[:])
                continue
            
            # Rastgele bir kesim noktası seç
            start, end = sorted(random.sample(range(len(route1)), 2))
            # Çocuk 1: parent1'in kesim parçası + parent2'nin kalanları
            child1_route = route1[start:end]
            child1_route += [dp for dp in route2 if dp not in child1_route]
            # Çocuk 2: parent2'nin kesim parçası + parent1'in kalanları
            child2_route = route2[start:end]
            child2_route += [dp for dp in route1 if dp not in child2_route]
            
            child1.append(child1_route)
            child2.append(child2_route)
        
        return child1, child2

    def _mutate(self, individual: List[List[int]]) -> List[List[int]]:
        """Rastgele iki teslimat noktasını yer değiştirir."""
        for route in individual:
            if random.random() < self.mutation_rate and len(route) > 1:
                i, j = random.sample(range(len(route)), 2)
                route[i], route[j] = route[j], route[i]
        return individual

    def run(self, current_time: str = "10:00") -> Tuple[List[List[int]], float]:
        """Genetik algoritmayı çalıştırır ve en iyi rotayı döndürür."""
        population = self.population[:]
        
        for _ in range(self.generations):
            new_population = []
            
            # Elitizm: En iyi bireyi koru
            best_individual = max(population, key=lambda ind: self._calculate_fitness(ind, current_time))
            new_population.append(best_individual)
            
            # Yeni popülasyon oluştur
            while len(new_population) < self.population_size:
                parent1 = self._tournament_selection(population)
                parent2 = self._tournament_selection(population)
                child1, child2 = self._crossover(parent1, parent2)
                child1 = self._mutate(child1)
                child2 = self._mutate(child2)
                new_population.extend([child1, child2])
            
            population = new_population[:self.population_size]
        
        # En iyi bireyi bul
        best_individual = max(population, key=lambda ind: self._calculate_fitness(ind, current_time))
        best_fitness = self._calculate_fitness(best_individual, current_time)
        
        return best_individual, best_fitness