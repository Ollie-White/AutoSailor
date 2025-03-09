import heapq
import math
import json
import time
import matplotlib.pyplot as plt
from vehicle_connection import VehicleConnection
from flask import Flask, jsonify, request
from flask_socketio import SocketIO
from flask_cors import CORS
from shapely.geometry import Point, Polygon, LineString
from scipy.spatial import distance


#Load configuration from external file
with open('environment-variables.json', 'r') as file:
    config = json.load(file)

USE_SIMULATOR = config['use_simulator']
SIMULATOR_START = config['starting_coordinates']
SIMULATOR_END = config['ending_coordinates']
RUN_API = config['run_api']

geofence = config['geofence']
geofence_polygon = Polygon(geofence)

# Define the buffer distance (in degrees)
BUFFER_DISTANCE = config['buffer_distance']

if RUN_API:
    app = Flask(__name__)
    socketio = SocketIO(app, cors_allowed_origins="*")
    CORS(app, origins="*")

def create_inner_buffer(geofence_polygon, buffer_distance):
    return geofence_polygon.buffer(-buffer_distance)

# Create the inner buffer polygon
inner_buffer = create_inner_buffer(geofence_polygon, BUFFER_DISTANCE)

def is_straight_line_possible(start, end):
    line = LineString([start, end])
    return inner_buffer.contains(line)

def heuristic(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def is_visible(point1, point2, inner_buffer):
    line = LineString([point1, point2])
    return inner_buffer.contains(line)

def create_visibility_graph(start, end, inner_buffer):
    inner_buffer_coords = list(inner_buffer.exterior.coords)
    vertices = inner_buffer_coords + [start, end]
    graph = {v: [] for v in vertices}

    for i, v1 in enumerate(vertices):
        for v2 in vertices[i+1:]:
            if is_visible(v1, v2, inner_buffer):
                dist = distance.euclidean(v1, v2)
                graph[v1].append((v2, dist))
                graph[v2].append((v1, dist))

    connection_count = sum(len(neighbors) for neighbors in graph.values())
    print(f"Total connections in visibility graph: {connection_count}")
    return graph

def a_star_search(graph, start, end):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}

    while open_set:
        current_f, current = heapq.heappop(open_set)

        if current == end:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for neighbor, dist in graph[current]:
            tentative_g_score = g_score[current] + dist

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor, end)
                heapq.heappush(open_set, (f_score, neighbor))

    return None

def find_path(start, end, inner_buffer):
    visibility_graph = create_visibility_graph(start, end, inner_buffer)
    #plot_visibility_graph(geofence, inner_buffer, start, end, visibility_graph)
    path = a_star_search(visibility_graph, start, end)
    return path

def create_waypoint_mission(start, end):
    if inner_buffer.contains(Point(start)) and inner_buffer.contains(Point(end)):
        if is_straight_line_possible(start, end):
            return [start, end]
        else:
            path = find_path(start, end, inner_buffer)
            if path:
                return path
            else:
                print("No valid path found within the inner buffer.")
                return None
    else:
        print("Start or end point is outside the safe inner buffer.")
        return None

def plot_geofence_and_inner_buffer(geofence, inner_buffer, start, end):
    # Extract geofence coordinates
    geofence_lats, geofence_lons = zip(*geofence)

    # Plot geofence
    plt.plot(geofence_lons, geofence_lats, 'b-', label='Geofence')

    # Plot inner buffer
    inner_buffer_coords = list(inner_buffer.exterior.coords)
    inner_buffer_lats, inner_buffer_lons = zip(*inner_buffer_coords)
    plt.plot(inner_buffer_lons, inner_buffer_lats, 'r--', label='Inner Buffer')

    plt.plot(start[1], start[0], 'go', label='Start')
    plt.plot(end[1], end[0], 'ro', label='End')

    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Geofence and Inner Buffer')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_geofence_and_path(geofence, inner_buffer, start, end, path):
    # Extract geofence coordinates
    geofence_lats, geofence_lons = zip(*geofence)

    # Plot geofence
    #plt.plot(geofence_lons, geofence_lats, 'b-', label='Geofence')

    # Plot inner buffer
    inner_buffer_coords = list(inner_buffer.exterior.coords)
    inner_buffer_lats, inner_buffer_lons = zip(*inner_buffer_coords)
    plt.plot(inner_buffer_lons, inner_buffer_lats, 'r--', label='Inner Buffer')

    # Plot start and end points
    plt.plot(start[1], start[0], 'go', label='Start')
    plt.plot(end[1], end[0], 'ro', label='End')

    # Plot path
    if path:
        path_lats, path_lons = zip(*path)
        plt.plot(path_lons, path_lats, 'g--', label='Path')

    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Geofence, Inner Buffer, and Path')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_visibility_graph(geofence, inner_buffer, start, end, graph):
    plot_geofence_and_inner_buffer(geofence, inner_buffer, start, end)
    for point, neighbors in graph.items():
        for neighbor, _ in neighbors:
            plt.plot([point[1], neighbor[1]], [point[0], neighbor[0]], 'y-', alpha=0.3)
    plt.show()

def navigate(destination):
    try:
        if destination is None:
            destination = (SIMULATOR_END[0], SIMULATOR_END[1])

        current_position = vehicle.get_current_position()

        if not inner_buffer.contains(Point(destination)):
            return {"error": "Destination is outside the safe inner buffer."}, 400
        #plot_geofence_and_inner_buffer(geofence, inner_buffer, current_position, destination)
        waypoints = create_waypoint_mission(current_position, destination)

        if not waypoints:
            return {"error": "No valid path found within the inner buffer."}, 400

        print("Waypoints:", waypoints)
        vehicle.send_waypoints(waypoints)
        #vehicle.verify_mission()
        vehicle.arm_vehicle()
        vehicle.set_mode("LOITER")
        time.sleep(1)
        vehicle.set_mode("AUTO")

        # Plot the geofence, inner buffer, and path
        #plot_geofence_and_path(geofence, inner_buffer, current_position, destination, waypoints)

        return {"message": "Navigation initiated successfully.", "waypoints": waypoints}, 200

    except Exception as e:
        return {"error": f"An error occurred during navigation: {str(e)}"}, 500

# Update the navigateBoatToPoint route to use the new navigate function
@app.route('/navigateBoatToPoint', methods=['POST'])
def navigateBoatToPoint():
    data = request.json
    destination = data.get('destination')

    if not destination or 'lat' not in destination or 'lng' not in destination:
        return jsonify({"error": "Invalid destination format."}), 400

    result, status_code = navigate((destination['lat'], destination['lng']))
    return jsonify(result), status_code

@socketio.on('connect')
def on_connect():
    print("Client connected")

def emit_position():
    while True:
        lat, lon = vehicle.get_current_position()
        socketio.emit('position_update', {'lon': lon, 'lat': lat})
        socketio.sleep(0.5)

if __name__ == "__main__":
    vehicle = VehicleConnection()
    if RUN_API:
        #app.run(debug=True)
        socketio.start_background_task(emit_position)
        socketio.run(app, host='0.0.0.0', port=5000)
    else:
        navigate(None)