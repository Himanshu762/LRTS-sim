from flask import Flask, render_template, request, jsonify
import folium
from simulation import SimulationManager
import threading
import time

app = Flask(__name__)

# Initialize simulation
DELHI_METRO_STATION = (28.6438, 77.1129)
simulation = SimulationManager(metro_station=DELHI_METRO_STATION)

def run_simulation():
    while True:
        try:
            if simulation.is_running:
                simulation.simulation_step()
            time.sleep(1)
        except Exception as e:
            print(f"Simulation error: {e}")
            continue

# Create a daemon thread that won't block the main application
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
    pickup = data.get('pickup')
    dropoffs = data.get('dropoffs')

    if not pickup or not dropoffs:
        return jsonify({'error': 'Invalid data provided.'}), 400

    # Use SimpleGraph for routing
    path_coords = []
    current_point = pickup
    
    for dropoff in dropoffs:
        start_node = simulation.G.nearest_node(current_point[0], current_point[1])
        end_node = simulation.G.nearest_node(dropoff[0], dropoff[1])
        
        path = simulation.G.shortest_path(start_node, end_node)
        path_coords.extend([
            (simulation.G.nodes[node]['y'], simulation.G.nodes[node]['x']) 
            for node in path
        ])
        current_point = dropoff

    return jsonify({'route': path_coords})

@app.route('/simulation_state')
def get_simulation_state():
    state = simulation.get_simulation_state()
    
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
                'route': [
                    {
                        'lat': simulation.G.nodes[node]['y'],
                        'lng': simulation.G.nodes[node]['x']
                    }
                    for node in (auto.route or [])
                ] if auto.route else [],
                'pickup_queue': [
                    {
                        'id': req.id,
                        'route': [
                            {
                                'lat': simulation.G.nodes[node]['y'],
                                'lng': simulation.G.nodes[node]['x']
                            }
                            for node in simulation.G.shortest_path(auto.current_node, req.pickup_node)
                        ]
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