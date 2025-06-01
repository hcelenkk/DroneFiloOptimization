from typing import List, Tuple
import heapq

class DeliveryPoint:
    def __init__(self, id: int, pos: tuple, weight: float, priority: int, time_window: tuple):
        self.id = id
        self.pos = pos
        self.weight = weight
        self.priority = priority
        self.time_window = time_window

def sort_deliveries_by_priority(deliveries: List[DeliveryPoint]) -> List[DeliveryPoint]:
    """Teslimat noktalarını öncelik sırasına göre sıralar (yüksek öncelik önce)."""
    heap = [(-dp.priority, dp.id, dp) for dp in deliveries]  # Negatif priority ve id ile Min-Heap
    heapq.heapify(heap)
    return [heapq.heappop(heap)[2] for _ in range(len(heap))]