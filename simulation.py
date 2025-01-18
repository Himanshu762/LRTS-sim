import numpy as np
import osmnx as ox
import networkx as nx
from dataclasses import dataclass
from typing import List, Tuple, Dict
import time
from enum import Enum
import random

class RideStatus(Enum):
    WAITING = "waiting"
    ASSIGNED = "assigned"
    IN_VEHICLE = "in_vehicle"
    COMPLETED = "completed"

class AutoStatus(Enum):
    IDLE = "idle"
    MOVING_TO_PICKUP = "moving_to_pickup"
    MOVING_TO_DROPOFF = "moving_to_dropoff"

@dataclass
class Request:
    id: int
    pickup_loc: Tuple[float, float]  # (lat, lon)
    pickup_node: int
    created_time: float
    status: RideStatus
    assigned_auto: int = None

@dataclass
class Auto:
    id: int
    current_loc: Tuple[float, float]  # (lat, lon)
    current_node: int
    status: AutoStatus
    capacity: int = 3
    current_passengers: List[Request] = None
    route: List[int] = None
    route_index: int = 0
    pickup_queue: List[Request] = None

    def __post_init__(self):
        if self.current_passengers is None:
            self.current_passengers = []
        if self.pickup_queue is None:
            self.pickup_queue = []
class SimulationManager:
    def __init__(self, metro_station: Tuple[float, float], path_length_km: float = 2.5, num_autos: int = 5):
        self.metro_station = metro_station
        self.path_length_km = path_length_km
        self.num_autos = 5
        
        # Initialize graph with larger area to ensure coverage
        self.G = ox.graph_from_point(metro_station, dist=5000, network_type="drive")
        self.G = ox.add_edge_speeds(self.G)
        self.G = ox.add_edge_travel_times(self.G)
        
        # Find metro station node
        self.metro_node = ox.distance.nearest_nodes(self.G, metro_station[1], metro_station[0])
        
        # Generate polygon vertices using network distance
        self.boundary_nodes = self.generate_boundary_nodes()
        
        # Initialize state
        self.requests = {}
        self.autos = self.initialize_autos()
        self.request_counter = 0
        self.enable_ride_sharing = False
        self.simulation_time = 0
        self.update_interval = 1
        self.is_running = False

    def initialize_autos(self) -> Dict[int, Auto]:
        autos = {}
        for i in range(self.num_autos):
            autos[i] = Auto(
                id=i,
                current_loc=self.metro_station,
                current_node=self.metro_node,
                status=AutoStatus.IDLE
            )
        return autos

    def generate_boundary_nodes(self):
        # Increase number of vertices for better precision
        angles = np.linspace(0, 2*np.pi, 8192, endpoint=False)  # 32 points instead of 8
        boundary_nodes = []
        
        # Get all nodes within max path length
        reachable_nodes = nx.single_source_dijkstra_path_length(
            self.G, 
            self.metro_node, 
            cutoff=self.path_length_km * 1000,  # Convert to meters
            weight='length'
        )
        
        target_distance = self.path_length_km * 1000  # Target distance in meters
        distance_tolerance = 50  # 50 meters tolerance
        
        for angle in angles:
            # Calculate target coordinates
            search_radius = 0.05  # Larger search radius in degrees
            target_x = self.metro_station[1] + np.cos(angle) * search_radius
            target_y = self.metro_station[0] + np.sin(angle) * search_radius
            
            # Find candidate nodes in the direction
            candidates = []
            for node, distance in reachable_nodes.items():
                if abs(distance - target_distance) <= distance_tolerance:
                    node_y = self.G.nodes[node]['y']
                    node_x = self.G.nodes[node]['x']
                    
                    # Calculate angle to this node
                    node_angle = np.arctan2(
                        node_y - self.metro_station[0],
                        node_x - self.metro_station[1]
                    )
                    
                    # Normalize angles for comparison
                    angle_diff = abs(np.mod(node_angle - angle + np.pi, 2*np.pi) - np.pi)
                    
                    if angle_diff < np.pi/16:  # Accept nodes within ±11.25 degrees
                        candidates.append((
                            node,
                            angle_diff,
                            abs(distance - target_distance)
                        ))
            
            if candidates:
                # Select best candidate based on both angle and distance accuracy
                best_node = min(candidates, key=lambda x: x[1] * 100 + x[2])[0]
                boundary_nodes.append(best_node)
        
        # Ensure the boundary is complete by connecting adjacent points
        final_boundary = []
        for i in range(len(boundary_nodes)):
            current_node = boundary_nodes[i]
            next_node = boundary_nodes[(i + 1) % len(boundary_nodes)]
            
            # Get path between adjacent boundary points
            try:
                path = nx.shortest_path(self.G, current_node, next_node, weight='length')
                final_boundary.extend(path[:-1])  # Exclude last node to avoid duplicates
            except nx.NetworkXNoPath:
                final_boundary.append(current_node)
        
        return final_boundary

    def generate_random_request(self):
        # Get all nodes within path length
        reachable_nodes = nx.single_source_dijkstra_path_length(
            self.G, 
            self.metro_node, 
            cutoff=self.path_length_km * 1000,  # Convert to meters
            weight='length'
        )
        
        # Filter nodes to ensure they're within path length limit
        valid_nodes = [node for node, dist in reachable_nodes.items() 
                      if dist <= self.path_length_km * 1000]
        
        if not valid_nodes:
            return None
        
        # Select random node
        pickup_node = random.choice(valid_nodes)
        pickup_lat = self.G.nodes[pickup_node]['y']
        pickup_lon = self.G.nodes[pickup_node]['x']
        
        request = Request(
            id=self.request_counter,
            pickup_loc=[pickup_lat, pickup_lon],
            pickup_node=pickup_node,
            created_time=self.simulation_time,
            status=RideStatus.WAITING
        )
        
        self.request_counter += 1
        return request

    def find_nearest_available_auto(self, request: Request, consider_ride_sharing: bool = True) -> Auto:
        min_distance = float('inf')
        nearest_auto = None
        assigned_pickups = set()

        # Track all assigned pickup points
        for auto in self.autos.values():
            for passenger in auto.current_passengers:
                assigned_pickups.add(passenger.pickup_node)
            for queued in auto.pickup_queue:
                assigned_pickups.add(queued.pickup_node)

        for auto in self.autos.values():
            # Check if auto is available based on status and capacity
            if auto.status == AutoStatus.IDLE or (
                consider_ride_sharing and 
                len(auto.current_passengers) + len(auto.pickup_queue) < 2 and
                request.pickup_node not in assigned_pickups
            ):
                try:
                    distance = nx.shortest_path_length(
                        self.G, 
                        auto.current_node, 
                        request.pickup_node, 
                        weight='travel_time'
                    )
                    if distance < min_distance:
                        min_distance = distance
                        nearest_auto = auto
                except nx.NetworkXNoPath:
                    continue

        return nearest_auto

    def check_request_proximity(self, request1: Request, request2: Request) -> float:
        try:
            return nx.shortest_path_length(
                self.G,
                request1.pickup_node,
                request2.pickup_node,
                weight='length'  # Use actual distance in meters
            )
        except nx.NetworkXNoPath:
            return float('inf')

    def process_requests(self):
        # Get all waiting requests
        waiting_requests = [r for r in self.requests.values() if r.status == RideStatus.WAITING]
        
        if not waiting_requests:
            return

        # Sort requests by creation time
        waiting_requests.sort(key=lambda x: x.created_time)

        # Process requests based on the number of waiting requests
        if len(waiting_requests) >= 4:
            # Case 1: 4th request after 1st pickup
            first_request = waiting_requests[0]
            if any(auto.current_passengers for auto in self.autos.values()):
                self.process_fourth_request(waiting_requests[3])
                
            # Case 2: 3rd & 4th request after 1st pickup
            elif len(waiting_requests) >= 2:
                self.process_third_fourth_requests(waiting_requests[2:4])
                
        # Case 3: Regular processing for other requests
        for request in waiting_requests:
            self.process_single_request(request)

    def process_fourth_request(self, request: Request):
        # Sort 3rd and 4th requests
        third_request = next(r for r in self.requests.values() 
                            if r.status == RideStatus.WAITING 
                            and r.created_time < request.created_time)
        
        distance = self.check_request_proximity(third_request, request)
        
        if distance <= 1000:  # 1km threshold
            # Try to assign to same auto
            auto = self.find_nearest_available_auto(third_request)
            if auto and len(auto.current_passengers) + len(auto.pickup_queue) < 2:
                self.assign_request_to_auto(request, auto)
        else:
            # Assign to nearest available auto
            auto = self.find_nearest_available_auto(request, consider_ride_sharing=False)
            if auto:
                self.assign_request_to_auto(request, auto)

    def process_third_fourth_requests(self, requests: List[Request]):
        if len(requests) < 2:
            return
        
        third_request, fourth_request = requests
        distance = self.check_request_proximity(third_request, fourth_request)
        
        # Check if requests can be served within 20 minutes
        try:
            total_time = nx.shortest_path_length(
                self.G,
                third_request.pickup_node,
                fourth_request.pickup_node,
                weight='travel_time'
            )
            
            if total_time <= 1200:  # 20 minutes in seconds
                # Assign to same auto if possible
                auto = self.find_nearest_available_auto(third_request)
                if auto and len(auto.current_passengers) + len(auto.pickup_queue) < 2:
                    self.assign_request_to_auto(third_request, auto)
                    self.assign_request_to_auto(fourth_request, auto)
                    return
        except nx.NetworkXNoPath:
            pass
        
        # Assign to separate autos
        for request in [third_request, fourth_request]:
            auto = self.find_nearest_available_auto(request, consider_ride_sharing=False)
            if auto:
                self.assign_request_to_auto(request, auto)

    def process_single_request(self, request: Request):
        auto = self.find_nearest_available_auto(request)
        if auto:
            self.assign_request_to_auto(request, auto)

    def assign_request_to_auto(self, request: Request, auto: Auto):
        request.status = RideStatus.ASSIGNED
        request.assigned_auto = auto.id
        
        if auto.status == AutoStatus.IDLE:
            auto.current_passengers.append(request)
            auto.status = AutoStatus.MOVING_TO_PICKUP
            auto.route = nx.shortest_path(
                self.G, 
                auto.current_node, 
                request.pickup_node, 
                weight='travel_time'
            )
            auto.route_index = 0
        else:
            auto.pickup_queue.append(request)

    def update_auto_location(self, auto: Auto):
        if not auto.route or auto.route_index >= len(auto.route):
            # If there are more pickups in queue, set route to next pickup
            if auto.status == AutoStatus.MOVING_TO_PICKUP and len(auto.pickup_queue) > 0:
                next_pickup = auto.pickup_queue.pop(0)
                auto.route = nx.shortest_path(self.G, auto.current_node, next_pickup.pickup_node, weight='travel_time')
                auto.route_index = 0
                return
            elif auto.status == AutoStatus.MOVING_TO_DROPOFF and len(auto.pickup_queue) > 0:
                # After dropping off, if more pickups are queued, go to next pickup
                auto.status = AutoStatus.MOVING_TO_PICKUP
                next_pickup = auto.pickup_queue[0]
                auto.route = nx.shortest_path(self.G, auto.current_node, next_pickup.pickup_node, weight='travel_time')
                auto.route_index = 0
                return
            return

        # Move to next node in route
        auto.current_node = auto.route[auto.route_index]
        auto.current_loc = (
            self.G.nodes[auto.current_node]['y'],
            self.G.nodes[auto.current_node]['x']
        )
        auto.route_index += 1

        # Check if route is completed
        if auto.route_index >= len(auto.route):
            if auto.status == AutoStatus.MOVING_TO_PICKUP:
                # Try to find an assigned passenger
                assigned_passengers = [r for r in auto.current_passengers if r.status == RideStatus.ASSIGNED]
                
                if assigned_passengers:
                    # Pick up passenger
                    request = assigned_passengers[0]
                    request.status = RideStatus.IN_VEHICLE
                    
                    if len(auto.pickup_queue) > 0:
                        # If more pickups queued, go to next pickup
                        next_pickup = auto.pickup_queue.pop(0)
                        auto.route = nx.shortest_path(self.G, auto.current_node, next_pickup.pickup_node, weight='travel_time')
                        auto.route_index = 0
                    else:
                        # No more pickups, head to metro station
                        auto.status = AutoStatus.MOVING_TO_DROPOFF
                        auto.route = nx.shortest_path(self.G, auto.current_node, self.metro_node, weight='travel_time')
                        auto.route_index = 0
                else:
                    # No assigned passengers found, return to metro station
                    auto.status = AutoStatus.MOVING_TO_DROPOFF
                    auto.route = nx.shortest_path(self.G, auto.current_node, self.metro_node, weight='travel_time')
                    auto.route_index = 0
            elif auto.status == AutoStatus.MOVING_TO_DROPOFF:
                # Drop off passengers
                for request in auto.current_passengers:
                    request.status = RideStatus.COMPLETED
                auto.current_passengers = []
                auto.status = AutoStatus.IDLE
                auto.route = None
                auto.pickup_queue = []
    def simulation_step(self):
        # Generate new requests
        if random.random() < 0.1:  # 10% chance of new request per step
            request = self.generate_random_request()
            self.requests[request.id] = request

        # Process all requests using new logic
        self.process_requests()

        # Update auto locations
        for auto in self.autos.values():
            self.update_auto_location(auto)

        self.simulation_time += self.update_interval
    # For capacity estimation, define a separate function that doesn't slow the simulation:
    def estimate_capacity(self):
        # Peak hours capacity estimation
        peak_hours = {
            '7AM': 1.2,  # 20% more demand
            '8AM': 1.5,  # 50% more demand
            '9AM': 1.2,
            '5PM': 1.3,
            '6PM': 1.6,  # Highest demand
            '7PM': 1.4,
            '8PM': 1.2
        }
        
        base_capacity = 12  # Base requests per hour per auto
        num_autos = 5
        avg_trip_time = 17.5  # Average trip time in minutes
        
        hourly_capacity = []
        for hour, demand_factor in peak_hours.items():
            capacity = int(base_capacity * num_autos * demand_factor)
            hourly_capacity.append(capacity)
        
        return {
            'hourly_capacity': hourly_capacity,
            'avg_trip_time': avg_trip_time
        }

    def estimate_trip_time(self, request1: Request, request2: Request = None) -> float:
        try:
            if request2:
                # Calculate combined trip time for shared ride
                pickup_to_pickup = nx.shortest_path_length(
                    self.G,
                    request1.pickup_node,
                    request2.pickup_node,
                    weight='travel_time'
                )
                return pickup_to_pickup + 900  # Add 15 minutes base time
            else:
                # Single trip estimation
                return 900  # 15 minutes base time
        except nx.NetworkXNoPath:
            return float('inf')
    def get_simulation_state(self):
        return {
            'time': self.simulation_time,
            'autos': self.autos,
            'requests': self.requests,
            'ride_sharing': self.enable_ride_sharing
        }