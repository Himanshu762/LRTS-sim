from typing import List, Tuple, Dict
import numpy as np
from haversine import haversine
import math
import xml.etree.ElementTree as ET
import requests
import os
import time

class SimpleGraph:
    def __init__(self, center: Tuple[float, float], radius_km: float):
        self.center = center
        self.radius_km = radius_km
        self.nodes = {}
        self.edges = {}
        self._load_or_download_osm_data()
        self._build_road_network()
        
    def _load_or_download_osm_data(self):
        cache_file = f"osm_cache_{self.center[0]}_{self.center[1]}_{self.radius_km}.osm"
        
        if not os.path.exists(cache_file):
            # Calculate bounding box
            lat_offset = self.radius_km / 111.32
            lon_offset = self.radius_km / (111.32 * math.cos(math.radians(self.center[0])))
            bbox = (
                self.center[1] - lon_offset,  # min lon
                self.center[0] - lat_offset,  # min lat
                self.center[1] + lon_offset,  # max lon
                self.center[0] + lat_offset   # max lat
            )
            
            # Use Overpass API instead of OSM API
            overpass_url = "http://overpass-api.de/api/interpreter"
            overpass_query = f"""
                [out:xml][timeout:25];
                (
                  way["highway"]({bbox[1]},{bbox[0]},{bbox[3]},{bbox[2]});
                  >;
                );
                out body;
            """
            
            try:
                response = requests.post(overpass_url, data=overpass_query)
                if response.status_code == 200:
                    with open(cache_file, 'w', encoding='utf-8') as f:
                        f.write(response.text)
                else:
                    raise Exception(f"Overpass API error: {response.status_code}")
            except Exception as e:
                print(f"Error downloading OSM data: {e}")
                self._generate_fallback_grid()
                return
                
        try:
            self.osm_tree = ET.parse(cache_file)
        except Exception as e:
            print(f"Error parsing OSM data: {e}")
            self._generate_fallback_grid()

    def _generate_fallback_grid(self):
        print("Generating fallback grid network...")
        # Create a basic grid if OSM data fails
        grid_size = 20
        lat_span = self.radius_km / 111.32
        lon_span = self.radius_km / (111.32 * math.cos(math.radians(self.center[0])))
        
        node_id = 0
        for i in range(grid_size):
            for j in range(grid_size):
                lat = self.center[0] - lat_span + (2 * lat_span * i / (grid_size - 1))
                lon = self.center[1] - lon_span + (2 * lon_span * j / (grid_size - 1))
                if haversine(self.center, (lat, lon)) <= self.radius_km:
                    self.nodes[node_id] = {'y': lat, 'x': lon, 'type': 'grid'}
                    node_id += 1
        
        # Create edges between adjacent nodes
        for node1 in self.nodes:
            self.edges[node1] = {}
            for node2 in self.nodes:
                if node1 != node2:
                    dist = haversine(
                        (self.nodes[node1]['y'], self.nodes[node1]['x']),
                        (self.nodes[node2]['y'], self.nodes[node2]['x'])
                    )
                    if dist < self.radius_km / 10:
                        self.edges[node1][node2] = dist

    def _build_road_network(self):
        root = self.osm_tree.getroot()
        node_dict = {}
        node_id = 0
        
        # First pass: collect all nodes
        for node in root.findall(".//node"):
            lat = float(node.get('lat'))
            lon = float(node.get('lon'))
            if haversine(self.center, (lat, lon)) <= self.radius_km:
                node_dict[node.get('id')] = (lat, lon)
                self.nodes[node_id] = {'y': lat, 'x': lon, 'osm_id': node.get('id')}
                node_id += 1
        
        # Second pass: create edges from ways
        for way in root.findall(".//way"):
            # Check if it's a road
            is_road = False
            for tag in way.findall("tag"):
                if tag.get('k') == 'highway':
                    is_road = True
                    road_type = tag.get('v')
                    break
            
            if is_road:
                nodes = way.findall("nd")
                for i in range(len(nodes) - 1):
                    node1_osm = nodes[i].get('ref')
                    node2_osm = nodes[i + 1].get('ref')
                    
                    if node1_osm in node_dict and node2_osm in node_dict:
                        # Find corresponding node IDs in our graph
                        node1 = next(k for k, v in self.nodes.items() 
                                   if v['osm_id'] == node1_osm)
                        node2 = next(k for k, v in self.nodes.items() 
                                   if v['osm_id'] == node2_osm)
                        
                        # Calculate distance and add edges
                        dist = haversine(
                            (self.nodes[node1]['y'], self.nodes[node1]['x']),
                            (self.nodes[node2]['y'], self.nodes[node2]['x'])
                        )
                        
                        # Initialize edge dictionaries if needed
                        if node1 not in self.edges:
                            self.edges[node1] = {}
                        if node2 not in self.edges:
                            self.edges[node2] = {}
                        
                        # Add bidirectional edges with weights based on road type
                        weight = self._get_road_weight(road_type)
                        self.edges[node1][node2] = dist * weight
                        self.edges[node2][node1] = dist * weight

    def _get_road_weight(self, road_type: str) -> float:
        weights = {
            'motorway': 0.8,
            'trunk': 0.9,
            'primary': 1.0,
            'secondary': 1.1,
            'tertiary': 1.2,
            'residential': 1.3,
            'service': 1.4
        }
        return weights.get(road_type, 1.3)

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