import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Dict
import time
from enum import Enum
import random
from routing import SimpleGraph

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
        self.metro_station = metro_station
        self.radius_km = 2.5
        self.num_autos = 6
        
        # Initialize graph using SimpleGraph instead of osmnx
        self.G = SimpleGraph(metro_station, radius_km)
        
        # Find metro station node
        self.metro_node = self.G.nearest_node(metro_station[0], metro_station[1])
        
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

    def generate_random_request(self):
        angle = random.uniform(0, 2 * np.pi)
        r = random.uniform(0, self.radius_km)
        
        # Convert to cartesian coordinates
        dx = r * np.cos(angle)
        dy = r * np.sin(angle)
        
        # Convert to lat/lon
        lat = self.metro_station[0] + (dy / 111)
        lon = self.metro_station[1] + (dx / (111 * np.cos(np.radians(self.metro_station[0]))))
        
        # Debug print
        print(f"Generated request at: [{lat}, {lon}]")
        
        pickup_node = self.G.nearest_node(lat, lon)
        
        request = Request(
            id=self.request_counter,
            pickup_loc=[lat, lon],
            pickup_node=pickup_node,
            created_time=self.simulation_time,
            status=RideStatus.WAITING
        )
        
        self.request_counter += 1
        return request

    def find_nearest_available_auto(self, request: Request):
        min_distance = float('inf')
        nearest_auto = None
        assigned_pickups = set()

        for auto in self.autos.values():
            for passenger in auto.current_passengers:
                assigned_pickups.add(passenger.pickup_node)
            for queued in auto.pickup_queue:
                assigned_pickups.add(queued.pickup_node)

        for auto in self.autos.values():
            if auto.status == AutoStatus.IDLE or (
                self.enable_ride_sharing and 
                len(auto.current_passengers) + len(auto.pickup_queue) < 2 and
                request.pickup_node not in assigned_pickups and
                auto.status == AutoStatus.MOVING_TO_PICKUP
            ):
                try:
                    path = self.G.shortest_path(auto.current_node, request.pickup_node)
                    distance = len(path)  # Use path length as distance metric
                    if distance < min_distance:
                        min_distance = distance
                        nearest_auto = auto
                except Exception:
                    continue

        return nearest_auto

    def update_auto_location(self, auto: Auto):
        if not auto.route or auto.route_index >= len(auto.route):
            if auto.status == AutoStatus.MOVING_TO_PICKUP and len(auto.pickup_queue) > 0:
                next_pickup = auto.pickup_queue.pop(0)
                auto.route = self.G.shortest_path(auto.current_node, next_pickup.pickup_node)
                auto.route_index = 0
                return
            elif auto.status == AutoStatus.MOVING_TO_DROPOFF and len(auto.pickup_queue) > 0:
                auto.status = AutoStatus.MOVING_TO_PICKUP
                next_pickup = auto.pickup_queue[0]
                auto.route = self.G.shortest_path(auto.current_node, next_pickup.pickup_node)
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
                        auto.route = self.G.shortest_path(auto.current_node, next_pickup.pickup_node)
                        auto.route_index = 0
                    else:
                        # No more pickups, head to metro station
                        auto.status = AutoStatus.MOVING_TO_DROPOFF
                        auto.route = self.G.shortest_path(auto.current_node, self.metro_node)
                        auto.route_index = 0
                else:
                    # No assigned passengers found, return to metro station
                    auto.status = AutoStatus.MOVING_TO_DROPOFF
                    auto.route = self.G.shortest_path(auto.current_node, self.metro_node)
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
        try:
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
                        auto.current_passengers.append(request)
                        auto.status = AutoStatus.MOVING_TO_PICKUP
                        auto.route = self.G.shortest_path(auto.current_node, request.pickup_node)
                        auto.route_index = 0
                    elif self.enable_ride_sharing:
                        auto.pickup_queue.append(request)

            # Update auto locations
            for auto in self.autos.values():
                self.update_auto_location(auto)

            self.simulation_time += self.update_interval
        except Exception as e:
            print(f"Error in simulation step: {e}")

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
        
        if max_passengers_per_auto == 1:
            # Single passenger mode
            total_capacity = int(num_autos * trips_per_hour * hours)
        else:
            # Ride sharing mode (2 passengers)
            # Add efficiency factor (0.8) due to routing overhead
            total_capacity = int(num_autos * trips_per_hour * hours * 1.8)
        
        return total_capacity

    def get_simulation_state(self):
        return {
            'time': self.simulation_time,
            'autos': self.autos,
            'requests': self.requests,
            'ride_sharing': self.enable_ride_sharing
        }