from flask import Flask, render_template, request, jsonify
import folium
import osmnx as ox
import networkx as nx
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from simulation import SimulationManager
import threading
import time

app = Flask(__name__)

# Initialize simulation
DELHI_METRO_STATION = (28.6438, 77.1129)  # Changed to Tagore Garden Metro Station
simulation = SimulationManager(metro_station=DELHI_METRO_STATION)

def run_simulation():
    while True:
        if simulation.is_running:
            simulation.simulation_step()
        time.sleep(1)

# Start simulation in background thread
simulation_thread = threading.Thread(target=run_simulation, daemon=True)
simulation_thread.start()

@app.route('/')
def index():
    # Render the main map page
    initial_location = [28.6438, 77.1129]  # Default location (Delhi)
    map = folium.Map(location=initial_location, zoom_start=12)
    return render_template('index.html', map=map._repr_html_())

@app.route('/optimize_route', methods=['POST'])
def optimize_route():
    data = request.json
    pickup = data.get('pickup')  # [lat, lon]
    dropoffs = data.get('dropoffs')  # List of [lat, lon]

    if not pickup or not dropoffs:
        return jsonify({'error': 'Invalid data provided.'}), 400

    # Create the road network
    G = ox.graph_from_point(pickup, dist=10000, network_type="drive")
    G = ox.add_edge_speeds(G)
    G = ox.add_edge_travel_times(G)

    # Find the nearest nodes to pickup and drop-off points
    pickup_node = ox.distance.nearest_nodes(G, pickup[1], pickup[0])
    dropoff_nodes = [ox.distance.nearest_nodes(G, point[1], point[0]) for point in dropoffs]
    all_nodes = [pickup_node] + dropoff_nodes

    # Create distance matrix
    def compute_distance(a, b):
        try:
            return nx.shortest_path_length(G, source=a, target=b, weight='travel_time')
        except:
            return float('inf')

    distance_matrix = [[compute_distance(a, b) for b in all_nodes] for a in all_nodes]

    # Solve VRP using OR-Tools
    manager = pywrapcp.RoutingIndexManager(len(all_nodes), 1, 0)
    model = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = model.RegisterTransitCallback(distance_callback)
    model.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    solution = model.SolveWithParameters(search_parameters)

    # Extract the solution
    route = []
    if solution:
        index = model.Start(0)
        while not model.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(model.NextVar(index))

    optimized_path = [all_nodes[i] for i in route]

    # Get the full path for visualization
    full_path_coords = []
    for i in range(len(optimized_path) - 1):
        sub_path = nx.shortest_path(G, source=optimized_path[i], target=optimized_path[i + 1], weight='travel_time')
        full_path_coords.extend([(G.nodes[node]['y'], G.nodes[node]['x']) for node in sub_path])

    return jsonify({'route': full_path_coords})

@app.route('/simulation_state')
def get_simulation_state():
    state = simulation.get_simulation_state()
    
    # Count picked up and completed requests
    picked_up_count = len([r for r in state['requests'].values() 
                          if r.status.value == 'in_vehicle'])
    completed_count = len([r for r in state['requests'].values() 
                          if r.status.value == 'completed'])
    
    return jsonify({
        'time': state['time'],
        'autos': [
            {
                'id': auto.id,
                'location': auto.current_loc,
                'status': auto.status.value,
                'passenger_ids': [p.id for p in auto.current_passengers],
                'route': [(simulation.G.nodes[node]['y'], simulation.G.nodes[node]['x']) 
                         for node in (auto.route or [])] if auto.route else [],
                'pickup_queue': [
                    {
                        'id': req.id,
                        'route': [(simulation.G.nodes[node]['y'], simulation.G.nodes[node]['x']) 
                                for node in nx.shortest_path(simulation.G, auto.current_node, req.pickup_node, weight='travel_time')]
                    } for req in auto.pickup_queue
                ] if auto.pickup_queue else []
            } for auto in state['autos'].values()
        ],
        'requests': [
            {
                'id': req.id,
                'pickup': req.pickup_loc,
                'status': req.status.value,
                'assigned_auto': req.assigned_auto
            } for req in state['requests'].values()
        ],
        'ride_sharing': state['ride_sharing'],
        'picked_up_count': picked_up_count,
        'completed_count': completed_count
    })

@app.route('/set_ride_sharing', methods=['POST'])
def set_ride_sharing():
    data = request.json
    simulation.enable_ride_sharing = data.get('enable_ride_sharing', False)
    return jsonify({'ride_sharing': simulation.enable_ride_sharing})

@app.route('/toggle_simulation', methods=['POST'])
def toggle_simulation():
    simulation.is_running = not simulation.is_running
    return jsonify({'is_running': simulation.is_running})

if __name__ == '__main__':
    app.run(debug=True)