from typing import List, Tuple, Dict
import numpy as np
from haversine import haversine
import requests
import polyline

class SimpleGraph:
    def __init__(self, center: Tuple[float, float], radius_km: float):
        self.center = center
        self.radius_km = radius_km
        self.nodes = self._generate_grid()
        self.edges = self._create_edges()
        self.cache = {}  # Cache for road paths
        
    def _generate_grid(self, grid_size: int = 20):
        # Create a sparser grid for road network nodes
        lat_span = self.radius_km / 111
        lon_span = self.radius_km / (111 * np.cos(np.radians(self.center[0])))
        
        lats = np.linspace(self.center[0] - lat_span, self.center[0] + lat_span, grid_size)
        lons = np.linspace(self.center[1] - lon_span, self.center[1] + lon_span, grid_size)
        
        nodes = {}
        node_id = 0
        for lat in lats:
            for lon in lons:
                if haversine(self.center, (lat, lon)) <= self.radius_km:
                    # Get the nearest road point using OSRM
                    road_point = self._snap_to_road(lat, lon)
                    nodes[node_id] = {'y': road_point[0], 'x': road_point[1]}
                    node_id += 1
        return nodes

    def _snap_to_road(self, lat: float, lon: float) -> Tuple[float, float]:
        try:
            url = f"http://router.project-osrm.org/nearest/v1/driving/{lon},{lat}?number=1"
            response = requests.get(url, timeout=5)  # Add timeout
            data = response.json()
            if data["code"] == "Ok":
                point = data["waypoints"][0]["location"]
                return (point[1], point[0])
        except Exception as e:
            print(f"Error snapping to road: {e}")
        return (lat, lon)  # Fallback to original point

    def _get_road_path(self, start: Tuple[float, float], end: Tuple[float, float]) -> List[Tuple[float, float]]:
        # Get actual road path using OSRM
        cache_key = f"{start}_{end}"
        if cache_key in self.cache:
            return self.cache[cache_key]

        url = f"http://router.project-osrm.org/route/v1/driving/{start[1]},{start[0]};{end[1]},{end[0]}?overview=full&geometries=polyline"
        try:
            response = requests.get(url)
            data = response.json()
            if data["code"] == "Ok":
                geometry = data["routes"][0]["geometry"]
                path = polyline.decode(geometry)
                self.cache[cache_key] = path
                return path
        except:
            pass
        return [start, end]  # Fallback to direct path

    def shortest_path(self, start_node: int, end_node: int) -> List[Tuple[float, float]]:
        start_coords = (self.nodes[start_node]['y'], self.nodes[start_node]['x'])
        end_coords = (self.nodes[end_node]['y'], self.nodes[end_node]['x'])
        return self._get_road_path(start_coords, end_coords)
    
    def nearest_node(self, lat: float, lon: float) -> int:
        min_dist = float('infinity')
        nearest = None
        for node_id, coords in self.nodes.items():
            dist = haversine((lat, lon), (coords['y'], coords['x']))
            if dist < min_dist:
                min_dist = dist
                nearest = node_id
        return nearest 