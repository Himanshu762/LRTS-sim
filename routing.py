from typing import List, Tuple, Dict
import numpy as np
from haversine import haversine
import math

class SimpleGraph:
    def __init__(self, center: Tuple[float, float], radius_km: float):
        self.center = center
        self.radius_km = radius_km
        self.nodes = self._generate_grid()
        self.edges = self._create_edges()
        
    def _generate_grid(self, grid_size: int = 30):
        # Create a grid of nodes
        lat_span = self.radius_km / 111
        lon_span = self.radius_km / (111 * np.cos(np.radians(self.center[0])))
        
        # Generate main roads (fewer, more structured points)
        nodes = {}
        node_id = 0
        
        # Create main arterial roads (north-south and east-west)
        road_count = 5
        for i in range(road_count):
            lat = self.center[0] - lat_span + (2 * lat_span * i / (road_count - 1))
            for j in range(grid_size):
                lon = self.center[1] - lon_span + (2 * lon_span * j / (grid_size - 1))
                if haversine(self.center, (lat, lon)) <= self.radius_km:
                    nodes[node_id] = {'y': lat, 'x': lon}
                    node_id += 1
        
        # Create connecting roads
        for i in range(grid_size):
            lat = self.center[0] - lat_span + (2 * lat_span * i / (grid_size - 1))
            for j in range(road_count):
                lon = self.center[1] - lon_span + (2 * lon_span * j / (road_count - 1))
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
                    # Connect only to nearby nodes (creating a road network)
                    if dist < self.radius_km / 10:
                        edges[node1][node2] = dist
        return edges

    def _heuristic(self, node1: int, node2: int) -> float:
        # A* heuristic using haversine distance
        return haversine(
            (self.nodes[node1]['y'], self.nodes[node1]['x']),
            (self.nodes[node2]['y'], self.nodes[node2]['x'])
        )

    def shortest_path(self, start_node: int, end_node: int) -> List[int]:
        # A* pathfinding algorithm
        open_set = {start_node}
        closed_set = set()
        
        came_from = {}
        g_score = {start_node: 0}
        f_score = {start_node: self._heuristic(start_node, end_node)}
        
        while open_set:
            current = min(open_set, key=lambda x: f_score.get(x, float('inf')))
            
            if current == end_node:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_node)
                return list(reversed(path))
            
            open_set.remove(current)
            closed_set.add(current)
            
            for neighbor in self.edges[current]:
                if neighbor in closed_set:
                    continue
                
                tentative_g_score = g_score[current] + self.edges[current][neighbor]
                
                if neighbor not in open_set:
                    open_set.add(neighbor)
                elif tentative_g_score >= g_score.get(neighbor, float('inf')):
                    continue
                
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + self._heuristic(neighbor, end_node)
        
        return [start_node, end_node]

    def nearest_node(self, lat: float, lon: float) -> int:
        min_dist = float('inf')
        nearest = None
        for node_id, coords in self.nodes.items():
            dist = haversine((lat, lon), (coords['y'], coords['x']))
            if dist < min_dist:
                min_dist = dist
                nearest = node_id
        return nearest 