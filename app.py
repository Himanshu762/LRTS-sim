from flask import Flask, render_template, request, jsonify
import folium
import osmnx as ox
import networkx as nx
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from simulation import SimulationManager
import os
from supabase import create_client, Client
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize Supabase with error handling
try:
    supabase_url = os.getenv("SUPABASE_URL")
    supabase_key = os.getenv("SUPABASE_KEY")
    
    if not supabase_url or not supabase_key:
        raise ValueError("Supabase credentials not found in environment variables")
        
    supabase = create_client(supabase_url, supabase_key)
except Exception as e:
    print(f"Error initializing Supabase: {e}")
    supabase = None

app = Flask(__name__)

# Initialize simulation with state management
def get_or_create_simulation():
    if not supabase:
        # Fallback to local-only simulation if Supabase is not available
        return SimulationManager(metro_station=(28.6438, 77.1129))
    
    try:
        response = supabase.table('simulation_state').select('*').execute()
        
        if response.data:
            state = response.data[0]
            simulation = SimulationManager(
                metro_station=(28.6438, 77.1129),
                state=state
            )
        else:
            simulation = SimulationManager(metro_station=(28.6438, 77.1129))
            supabase.table('simulation_state').insert({
                'time': simulation.simulation_time,
                'request_counter': simulation.request_counter,
                'enable_ride_sharing': simulation.enable_ride_sharing
            }).execute()
    except Exception as e:
        print(f"Error accessing Supabase: {e}")
        simulation = SimulationManager(metro_station=(28.6438, 77.1129))
    
    return simulation

@app.route('/')
def index():
    # Render the main map page
    initial_location = [28.6438, 77.1129]  # Tagore Garden location
    map_obj = folium.Map(location=initial_location, zoom_start=12)
    
    # Add the circular zone to the map
    folium.Circle(
        location=initial_location,
        radius=4000,  # 4 km
        color='blue',
        fill=True,
        fill_opacity=0.2
    ).add_to(map_obj)
    
    # Add metro station marker
    folium.CircleMarker(
        location=initial_location,
        radius=8,
        color='#FFD700',  # Gold border
        fill=True,
        fill_color='#FF4500',  # Orange-Red fill
        fill_opacity=1.0,
        weight=2,
        popup='Tagore Garden Metro Station'
    ).add_to(map_obj)
    
    return render_template('index.html', map=map_obj._repr_html_())

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
    simulation = get_or_create_simulation()
    state = simulation.get_simulation_state()
    
    if supabase:
        try:
            supabase.table('simulation_state').update({
                'time': state['time'],
                'request_counter': simulation.request_counter,
                'enable_ride_sharing': simulation.enable_ride_sharing
            }).eq('id', 1).execute()
        except Exception as e:
            print(f"Error updating Supabase: {e}")
    
    return jsonify(state)

@app.route('/toggle_ride_sharing', methods=['POST'])
def toggle_ride_sharing():
    simulation = get_or_create_simulation()
    data = request.json
    simulation.enable_ride_sharing = data.get('enabled', False)
    
    # Update in Supabase
    supabase.table('simulation_state').update({
        'enable_ride_sharing': simulation.enable_ride_sharing
    }).eq('id', 1).execute()
    
    return jsonify({'success': True})

if __name__ == '__main__':
    app.run(debug=True)