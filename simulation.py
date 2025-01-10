import numpy as np
import osmnx as ox
import networkx as nx
from dataclasses import dataclass
from typing import List, Tuple, Dict
import time
from enum import Enum
import random
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

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
    pickup_loc: Tuple[float, float]
    pickup_node: int
    dropoff_loc: Tuple[float, float]
    dropoff_node: int
    created_time: float
    status: RideStatus
    assigned_auto: int = None

@dataclass
class Auto:
    id: int
    current_loc: Tuple[float, float]
    current_node: int
    status: AutoStatus
    capacity: int = 4
    current_passengers: List[Request] = None
    pickup_route: List[int] = None
    dropoff_route: List[int] = None
    route_index: int = 0
    is_pickup_phase: bool = True

    def __post_init__(self):
        if self.current_passengers is None:
            self.current_passengers = []

class SimulationManager:
    def __init__(self, metro_station: Tuple[float, float], radius_km: float = 4.0, num_autos: int = 30):
        self.metro_station = metro_station
        self.radius_km = radius_km
        self.num_autos = num_autos
        
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
        self.max_passengers = 2  # Maximum number of passengers for ride-sharing
        self.simulation_time = 0
        self.update_interval = 0.2  # Reduced to 0.2 seconds for faster animation

    def initialize_autos(self) -> Dict[int, Auto]:
        autos = {}
        for i in range(self.num_autos):
            autos[i] = Auto(
                id=i,
                current_loc=(self.G.nodes[self.metro_node]['y'], self.G.nodes[self.metro_node]['x']),
                current_node=self.metro_node,
                status=AutoStatus.IDLE,
                pickup_route=None,
                dropoff_route=None,
                is_pickup_phase=True
            )
        return autos

    def generate_random_request(self):
        # Generate random pickup point
        angle = random.uniform(0, 2 * np.pi)
        distance = random.uniform(0, self.radius_km)
        
        pickup_lat = self.metro_station[0] + (distance * np.cos(angle) / 111)
        pickup_lon = self.metro_station[1] + (distance * np.sin(angle) / (111 * np.cos(self.metro_station[0] * np.pi / 180)))
        
        # Generate random dropoff point
        angle = random.uniform(0, 2 * np.pi)
        distance = random.uniform(0, self.radius_km)
        
        dropoff_lat = self.metro_station[0] + (distance * np.cos(angle) / 111)
        dropoff_lon = self.metro_station[1] + (distance * np.sin(angle) / (111 * np.cos(self.metro_station[0] * np.pi / 180)))
        
        pickup_node = ox.distance.nearest_nodes(self.G, pickup_lon, pickup_lat)
        dropoff_node = ox.distance.nearest_nodes(self.G, dropoff_lon, dropoff_lat)
        
        return Request(
            id=self.request_counter,
            pickup_loc=(pickup_lat, pickup_lon),
            pickup_node=pickup_node,
            dropoff_loc=(dropoff_lat, dropoff_lon),
            dropoff_node=dropoff_node,
            created_time=self.simulation_time,
            status=RideStatus.WAITING
        )

    def find_nearest_available_autos(self, request: Request, max_pickups: int) -> List[Auto]:
        autos_available = []
        
        for auto in self.autos.values():
            if (auto.status == AutoStatus.IDLE or 
                (self.enable_ride_sharing and 
                 len(auto.current_passengers) < self.max_passengers and 
                 auto.status == AutoStatus.MOVING_TO_DROPOFF)):
                try:
                    # Check if path exists
                    path = nx.shortest_path(self.G, auto.current_node, request.pickup_node, weight='travel_time')
                    if path:
                        distance = sum(self.G[path[i]][path[i+1]][0]['travel_time'] 
                                     for i in range(len(path)-1))
                        autos_available.append((auto, distance))
                except nx.NetworkXNoPath:
                    continue
        
        # Sort by distance and return top autos
        return [auto for auto, _ in sorted(autos_available, key=lambda x: x[1])[:max_pickups]]

    def update_auto_location(self, auto: Auto):
        current_route = auto.pickup_route if auto.is_pickup_phase else auto.dropoff_route
        
        if not current_route or auto.route_index >= len(current_route):
            return

        # Move to next node in route
        auto.current_node = current_route[auto.route_index]
        auto.current_loc = (
            self.G.nodes[auto.current_node]['y'],
            self.G.nodes[auto.current_node]['x']
        )
        auto.route_index += 1

        # Check if current route is completed
        if auto.route_index >= len(current_route):
            if auto.is_pickup_phase:
                # Update passenger status
                for request in auto.current_passengers:
                    if request.status == RideStatus.ASSIGNED:
                        request.status = RideStatus.IN_VEHICLE
                
                # Create route to metro station or next pickup
                if self.enable_ride_sharing and len(auto.current_passengers) < self.max_passengers:
                    # Check for nearby waiting requests
                    waiting_requests = [r for r in self.requests.values() 
                                     if r.status == RideStatus.WAITING]
                    if waiting_requests:
                        next_pickup = min(waiting_requests, 
                                        key=lambda r: nx.shortest_path_length(self.G, 
                                                                            auto.current_node, 
                                                                            r.pickup_node, 
                                                                            weight='travel_time'))
                        auto.pickup_route = nx.shortest_path(self.G, auto.current_node, 
                                                           next_pickup.pickup_node, 
                                                           weight='travel_time')
                        auto.route_index = 0
                        return
                
                # No more pickups, head to metro
                auto.dropoff_route = nx.shortest_path(self.G, auto.current_node, 
                                                    self.metro_node, weight='travel_time')
                auto.route_index = 0
                auto.is_pickup_phase = False
                auto.status = AutoStatus.MOVING_TO_DROPOFF
                
            else:
                # Complete the ride
                for request in auto.current_passengers:
                    request.status = RideStatus.COMPLETED
                auto.current_passengers = []
                auto.status = AutoStatus.IDLE
                auto.pickup_route = None
                auto.dropoff_route = None
                auto.is_pickup_phase = True

    def simulation_step(self):
        # Generate new requests randomly
        if random.random() < 0.1:
            request = self.generate_random_request()
            # Verify the pickup and dropoff nodes are reachable from metro
            try:
                if (nx.has_path(self.G, self.metro_node, request.pickup_node) and 
                    nx.has_path(self.G, request.pickup_node, self.metro_node)):
                    self.request_counter += 1
                    self.requests[request.id] = request
            except nx.NetworkXNoPath:
                return

        # Process waiting requests
        waiting_requests = [r for r in self.requests.values() if r.status == RideStatus.WAITING]
        for request in waiting_requests:
            available_autos = self.find_nearest_available_autos(request, 
                max_pickups=1 if not self.enable_ride_sharing else self.max_passengers)
            
            if available_autos:
                auto = available_autos[0]
                try:
                    pickup_path = nx.shortest_path(self.G, auto.current_node, 
                                                request.pickup_node, weight='travel_time')
                    request.status = RideStatus.ASSIGNED
                    request.assigned_auto = auto.id
                    auto.current_passengers.append(request)
                    
                    if auto.status == AutoStatus.IDLE:
                        auto.status = AutoStatus.MOVING_TO_PICKUP
                        auto.pickup_route = pickup_path
                        auto.route_index = 0
                        auto.is_pickup_phase = True
                except nx.NetworkXNoPath:
                    continue

        # Update auto locations
        for auto in self.autos.values():
            self.update_auto_location(auto)

        self.simulation_time += self.update_interval

    def get_simulation_state(self):
        return {
            'time': self.simulation_time,
            'autos': [
                {
                    'id': auto.id,
                    'location': auto.current_loc,
                    'status': auto.status.value,
                    'passenger_ids': [p.id for p in auto.current_passengers],
                    'route': [(self.G.nodes[node]['y'], self.G.nodes[node]['x']) 
                             for node in (auto.pickup_route if auto.is_pickup_phase else auto.dropoff_route) or []]
                } for auto in self.autos.values()
            ],
            'requests': [
                {
                    'id': req.id,
                    'pickup': req.pickup_loc,
                    'dropoff': req.dropoff_loc,
                    'status': req.status.value,
                    'assigned_auto': req.assigned_auto
                } for req in self.requests.values()
            ],
            'ride_sharing': self.enable_ride_sharing,
            'picked_up_count': len([r for r in self.requests.values() if r.status == RideStatus.IN_VEHICLE]),
            'completed_count': len([r for r in self.requests.values() if r.status == RideStatus.COMPLETED])
        } 

    def create_optimized_route(self, start_node: int, target_nodes: List[int]) -> List[int]:
        if not target_nodes:
            return []
        
        # Include start node and metro station in the nodes list
        all_nodes = [start_node] + target_nodes + [self.metro_node]
        
        # Create distance matrix
        def compute_distance(a, b):
            try:
                return nx.shortest_path_length(self.G, source=a, target=b, weight='travel_time')
            except:
                return float('inf')
        
        distance_matrix = [[compute_distance(a, b) for b in all_nodes] for a in all_nodes]
        
        # Setup OR-Tools routing model
        manager = pywrapcp.RoutingIndexManager(len(all_nodes), 1, 0)  # 1 vehicle, starts at index 0
        routing = pywrapcp.RoutingModel(manager)
        
        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return int(distance_matrix[from_node][to_node] * 10)  # Convert to integers for OR-Tools
        
        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        
        # Set first solution strategy
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        search_parameters.time_limit.seconds = 1  # Limit search time to 1 second
        
        # Solve
        solution = routing.SolveWithParameters(search_parameters)
        
        if not solution:
            # Fallback to simple path if optimization fails
            route = []
            current = start_node
            remaining = target_nodes.copy()
            
            while remaining:
                next_node = min(remaining, 
                    key=lambda n: nx.shortest_path_length(self.G, current, n, weight='travel_time'))
                path = nx.shortest_path(self.G, current, next_node, weight='travel_time')
                route.extend(path[1:])
                current = next_node
                remaining.remove(next_node)
            
            final_path = nx.shortest_path(self.G, current, self.metro_node, weight='travel_time')
            route.extend(final_path[1:])
            return route
        
        # Extract the optimized route
        optimized_route = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            next_node_index = manager.IndexToNode(solution.Value(routing.NextVar(index)))
            if next_node_index < len(all_nodes):  # Check if we're not at the end
                # Get the actual path between these nodes
                path = nx.shortest_path(self.G, all_nodes[node_index], all_nodes[next_node_index], 
                                      weight='travel_time')
                optimized_route.extend(path[1:])  # Exclude start node to avoid duplicates
            index = solution.Value(routing.NextVar(index))
        
        return optimized_route 