from haversine import haversine
from typing import List, Tuple
import numpy as np

class SimpleGraph:
    def __init__(self, center: Tuple[float, float], radius_km: float):
        self.center = center
        self.radius_km = radius_km
        self.nodes = self._generate_grid()
        self.edges = self._create_edges()
        
    def _generate_grid(self, grid_size: int = 50):
        # Create a grid of nodes within radius
        lat_span = self.radius_km / 111  # Approximate degrees
        lon_span = self.radius_km / (111 * np.cos(np.radians(self.center[0])))
        
        lats = np.linspace(self.center[0] - lat_span, self.center[0] + lat_span, grid_size)
        lons = np.linspace(self.center[1] - lon_span, self.center[1] + lon_span, grid_size)
        
        nodes = {}
        node_id = 0
        for lat in lats:
            for lon in lons:
                if haversine(self.center, (lat, lon)) <= self.radius_km:
                    nodes[node_id] = {'y': lat, 'x': lon}
                    node_id += 1
        return nodes
    
    def _create_edges(self):
        edges = {}
        for node1 in self.nodes:
            edges[node1] = {}
            for node2 in self.nodes:
                if node1 != node2:
                    dist = haversine(
                        (self.nodes[node1]['y'], self.nodes[node1]['x']),
                        (self.nodes[node2]['y'], self.nodes[node2]['x'])
                    )
                    if dist < self.radius_km / 10:  # Connect only nearby nodes
                        edges[node1][node2] = dist
        return edges
    
    def shortest_path(self, start_node: int, end_node: int) -> List[int]:
        # Simple Dijkstra's algorithm
        distances = {node: float('infinity') for node in self.nodes}
        distances[start_node] = 0
        previous = {node: None for node in self.nodes}
        unvisited = set(self.nodes.keys())
        
        while unvisited:
            current = min(unvisited, key=lambda x: distances[x])
            if current == end_node:
                break
                
            unvisited.remove(current)
            
            for neighbor, dist in self.edges[current].items():
                if neighbor in unvisited:
                    new_dist = distances[current] + dist
                    if new_dist < distances[neighbor]:
                        distances[neighbor] = new_dist
                        previous[neighbor] = current
        
        # Reconstruct path
        path = []
        current = end_node
        while current is not None:
            path.append(current)
            current = previous[current]
        return list(reversed(path))
    
    def nearest_node(self, lat: float, lon: float) -> int:
        min_dist = float('infinity')
        nearest = None
        for node_id, coords in self.nodes.items():
            dist = haversine((lat, lon), (coords['y'], coords['x']))
            if dist < min_dist:
                min_dist = dist
                nearest = node_id
        return nearest 