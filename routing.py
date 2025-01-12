from typing import List, Tuple, Dict
import numpy as np
from haversine import haversine
import math

class SimpleGraph:
    def __init__(self, center: Tuple[float, float], radius_km: float):
        self.center = center
        self.radius_km = radius_km
        self.nodes = {}
        self.edges = {}
        self._generate_road_network()
        
    def _generate_road_network(self):
        # Parameters for road network
        main_road_spacing = self.radius_km / 4  # Major roads every 1km
        minor_road_spacing = self.radius_km / 8  # Minor roads every 500m
        
        # Convert to lat/lon spans
        lat_main = main_road_spacing / 111
        lon_main = main_road_spacing / (111 * np.cos(np.radians(self.center[0])))
        lat_minor = minor_road_spacing / 111
        lon_minor = minor_road_spacing / (111 * np.cos(np.radians(self.center[0])))
        
        node_id = 0
        
        # Generate main roads (grid)
        for i in range(-4, 5):  # Main north-south roads
            lat = self.center[0] + (i * lat_main)
            for j in np.arange(-4, 4.1, 0.5):  # More points along the road
                lon = self.center[1] + (j * lon_main)
                if haversine(self.center, (lat, lon)) <= self.radius_km:
                    self.nodes[node_id] = {'y': lat, 'x': lon, 'type': 'main'}
                    node_id += 1
        
        # Generate minor roads
        for i in np.arange(-8, 8.1, 0.5):  # Minor east-west roads
            lat = self.center[0] + (i * lat_minor)
            for j in range(-8, 9):
                lon = self.center[1] + (j * lon_minor)
                if haversine(self.center, (lat, lon)) <= self.radius_km:
                    self.nodes[node_id] = {'y': lat, 'x': lon, 'type': 'minor'}
                    node_id += 1
        
        # Create edges (connections between nodes)
        for node1 in self.nodes:
            self.edges[node1] = {}
            for node2 in self.nodes:
                if node1 != node2:
                    dist = haversine(
                        (self.nodes[node1]['y'], self.nodes[node1]['x']),
                        (self.nodes[node2]['y'], self.nodes[node2]['x'])
                    )
                    # Connect nodes if they're close and form a grid-like pattern
                    if dist < self.radius_km / 16:  # Shorter connections for grid-like pattern
                        # Add weight based on road type
                        weight = 1.0
                        if self.nodes[node1]['type'] == 'minor' or self.nodes[node2]['type'] == 'minor':
                            weight = 1.2  # Minor roads are slower
                        self.edges[node1][node2] = dist * weight

    def shortest_path(self, start_node: int, end_node: int) -> List[int]:
        # A* pathfinding with road type preferences
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

    def _heuristic(self, node1: int, node2: int) -> float:
        return haversine(
            (self.nodes[node1]['y'], self.nodes[node1]['x']),
            (self.nodes[node2]['y'], self.nodes[node2]['x'])
        )

    def nearest_node(self, lat: float, lon: float) -> int:
        min_dist = float('inf')
        nearest = None
        for node_id, coords in self.nodes.items():
            dist = haversine((lat, lon), (coords['y'], coords['x']))
            if dist < min_dist:
                min_dist = dist
                nearest = node_id
        return nearest 