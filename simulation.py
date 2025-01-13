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
    def __init__(self, metro_station: Tuple[float, float], radius_km: float = 4.0, num_autos: int = 5):
        # Use 2.5km radius and 6 autos
        self.metro_station = metro_station
        self.radius_km = 2.5
        self.num_autos = 6
        
        # Initialize graph
        self.G = ox.graph_from_point(metro_station, dist=radius_km * 1000, network_type="drive")
        self.G = ox.add_edge_speeds(self.G)
        self.G = ox.add_edge_travel_times(self.G)
        
        # Find metro station node
        self.metro_node = ox.distance.nearest_nodes(self.G, metro_station[1], metro_station[0])
        
        # Initialize state
        self.requests = {}
        self.autos = self.initialize_autos()
        self.request_counter = 0
        self.enable_ride_sharing = False
        self.simulation_time = 0
        self.update_interval = 1  # seconds
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
    def generate_random_request(self) -> Request:
        # Generate random point within radius
        angle = random.uniform(0, 2 * np.pi)
        r = random.uniform(0, self.radius_km)
        
        # Convert to cartesian coordinates
        dx = r * np.cos(angle)
        dy = r * np.sin(angle)
        
        # Convert to lat/lon
        lat = self.metro_station[0] + (dy / 111)  # Approximate conversion
        lon = self.metro_station[1] + (dx / (111 * np.cos(np.radians(self.metro_station[0]))))
        
        # Debug print
        print(f"Generated request at: [{lat}, {lon}]")
        
        pickup_node = ox.distance.nearest_nodes(self.G, lon, lat)
        
        request = Request(
            id=self.request_counter,
            pickup_loc=[lat, lon],  # Changed to list format for JSON serialization
            pickup_node=pickup_node,
            created_time=self.simulation_time,
            status=RideStatus.WAITING
        )
        
        self.request_counter += 1
        return request
    def find_nearest_available_auto(self, request: Request) -> Auto:
        min_score = float('inf')
        nearest_auto = None
        assigned_pickups = set()

        # Calculate request's distance to metro
        try:
            request_metro_dist = nx.shortest_path_length(
                self.G,
                request.pickup_node,
                self.metro_node,
                weight='travel_time'
            )
        except nx.NetworkXNoPath:
            request_metro_dist = float('inf')

        # Collect all assigned pickup points
        for auto in self.autos.values():
            for passenger in auto.current_passengers:
                assigned_pickups.add(passenger.pickup_node)
            for queued in auto.pickup_queue:
                assigned_pickups.add(queued.pickup_node)

        # Find best available auto based on combined score
        for auto in self.autos.values():
            if auto.status == AutoStatus.IDLE or (
                self.enable_ride_sharing and 
                len(auto.current_passengers) + len(auto.pickup_queue) < 2 and
                request.pickup_node not in assigned_pickups and
                auto.status == AutoStatus.MOVING_TO_PICKUP
            ):
                try:
                    # Calculate auto's current distance to metro
                    auto_metro_dist = nx.shortest_path_length(
                        self.G,
                        auto.current_node,
                        self.metro_node,
                        weight='travel_time'
                    )
                    
                    # Calculate distance from auto to request
                    auto_request_dist = nx.shortest_path_length(
                        self.G,
                        auto.current_node,
                        request.pickup_node,
                        weight='travel_time'
                    )

                    # Calculate efficiency score (lower is better)
                    # Consider:
                    # 1. Distance from auto to request
                    # 2. Distance from request to metro
                    # 3. Auto's current distance to metro
                    # 4. Current passenger count
                    passenger_factor = 1.0 if len(auto.current_passengers) == 0 else 0.7
                    
                    score = (
                        (0.3 * auto_request_dist) +  # Weight for pickup distance
                        (0.3 * request_metro_dist) +  # Weight for request-to-metro distance
                        (0.3 * auto_metro_dist) +    # Weight for auto's distance to metro
                        (0.1 * passenger_factor)      # Small weight for passenger optimization
                    )

                    if score < min_score:
                        min_score = score
                        nearest_auto = auto

                except nx.NetworkXNoPath:
                    continue

        return nearest_auto
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

        # Process waiting requests
        waiting_requests = [r for r in self.requests.values() if r.status == RideStatus.WAITING]
        for request in waiting_requests:
            auto = self.find_nearest_available_auto(request)
            if auto:
                request.status = RideStatus.ASSIGNED
                request.assigned_auto = auto.id
                
                if auto.status == AutoStatus.IDLE:
                    # First pickup for idle auto
                    auto.current_passengers.append(request)
                    auto.status = AutoStatus.MOVING_TO_PICKUP
                    auto.route = nx.shortest_path(
                        self.G, auto.current_node, request.pickup_node, weight='travel_time'
                    )
                    auto.route_index = 0
                elif self.enable_ride_sharing:
                    # Add to pickup queue for ride sharing
                    auto.pickup_queue.append(request)
                    # Calculate and store the route for visualization
                    if len(auto.pickup_queue) == 1:
                        request.route = nx.shortest_path(
                            self.G, auto.current_node, request.pickup_node, weight='travel_time'
                        )
        # Update auto locations
        for auto in self.autos.values():
            self.update_auto_location(auto)

        self.simulation_time += self.update_interval
    # For capacity estimation, define a separate function that doesn't slow the simulation:
    def estimate_capacity(self, hours=3, max_passengers_per_auto=1):
        # Constants for calculation
        num_autos = 30  # Total autos in 3-hour period
        avg_speed_kmh = 20  # Average speed in km/h
        avg_pickup_time = 2  # Minutes per pickup
        avg_dropoff_time = 2  # Minutes per dropoff
        
        # Calculate average trip metrics
        avg_trip_distance = self.radius_km * 1.4  # Factor in non-direct routes
        avg_trip_time_mins = (avg_trip_distance / avg_speed_kmh) * 60  # Convert to minutes
        
        # Total time per trip including pickup and dropoff
        total_trip_time = avg_trip_time_mins + avg_pickup_time + avg_dropoff_time
        
        # Calculate trips per hour per auto
        trips_per_hour = 60 / total_trip_time
        
        # Calculate total capacity
        total_trips = trips_per_hour * num_autos * hours
        total_passengers = total_trips * max_passengers_per_auto
        
        return {
            'total_trips': round(total_trips),
            'total_passengers': round(total_passengers),
            'trips_per_hour': round(trips_per_hour, 1),
            'avg_trip_time_mins': round(total_trip_time, 1)
        }
    def get_simulation_state(self):
        return {
            'time': self.simulation_time,
            'autos': self.autos,
            'requests': self.requests,
            'ride_sharing': self.enable_ride_sharing
        }