from flask import Flask, jsonify, request
from shapely.geometry import Point, Polygon, LineString
import heapq
import time
import matplotlib.pyplot as plt
from scipy.spatial import distance
import math
import json



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

if USE_SIMULATOR:
    from pymavlink import mavutil

if RUN_API:
    app = Flask(__name__)

def create_inner_buffer(geofence_polygon, buffer_distance):
    return geofence_polygon.buffer(-buffer_distance)

# Create the inner buffer polygon
inner_buffer = create_inner_buffer(geofence_polygon, BUFFER_DISTANCE)

class VehicleConnection:
    def __init__(self):
        if USE_SIMULATOR:
            self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
            print("Waiting for heartbeat...")
            self.connection.wait_heartbeat()
            print(f"Connected to system {self.connection.target_system}, component {self.connection.target_component}")
        else:
            self.connection = None

    def get_current_position(self):
        if USE_SIMULATOR:
            msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
        else:
            # Replace with your desired offline coordinates
            lat, lon = SIMULATOR_START
        return lat, lon

    def send_waypoints(self, waypoints):
        if USE_SIMULATOR:
            print("Sending waypoints...")
            self.connection.mav.mission_count_send(self.connection.target_system, self.connection.target_component, len(waypoints))

            for i, (lat, lon) in enumerate(waypoints):
                time.sleep(1)  # Prevent packet loss
                self.connection.mav.mission_item_send(
                    self.connection.target_system, self.connection.target_component, i,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                    lat, lon, 0
                )
                print(f"Sent waypoint {i}: ({lat}, {lon}, 0)")
        else:
            print("Offline mode: Waypoints would be sent here in simulator mode.")
            for i, (lat, lon) in enumerate(waypoints):
                print(f"Waypoint {i}: ({lat}, {lon}, 0)")

    def set_mode(self, mode):
        if USE_SIMULATOR:
            mode_id = self.connection.mode_mapping()[mode]
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
        print(f"Mode set to {mode}")

    def arm_vehicle(self):
        if USE_SIMULATOR:
            print("Arming vehicle...")
            self.connection.arducopter_arm()
            time.sleep(1)
        print("Vehicle armed!")

    def verify_mission(self):
        if USE_SIMULATOR:
            print("Verifying mission upload...")
            msg = self.connection.recv_match(type='MISSION_ACK', blocking=True)
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print("Mission uploaded successfully!")
            else:
                print(f"Mission upload failed: {msg}")
        else:
            print("Offline mode: Mission verification would occur here in simulator mode.")

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
    plt.plot(geofence_lons, geofence_lats, 'b-', label='Geofence')

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
    if destination is None:
        destination = (SIMULATOR_END[0],SIMULATOR_END[1])
    current_position = vehicle.get_current_position()
    if inner_buffer.contains(Point(destination)):
        plot_geofence_and_inner_buffer(geofence, inner_buffer, current_position, destination)
        waypoints = create_waypoint_mission(current_position, destination)
        if waypoints:
            print("Waypoints:", waypoints)
            vehicle.send_waypoints(waypoints)
            vehicle.verify_mission()
            vehicle.arm_vehicle()
            vehicle.set_mode("AUTO")
            # Plot the geofence, inner buffer, and path
            plot_geofence_and_path(geofence, inner_buffer, current_position, destination, waypoints)
        else:
            print("No valid path found within the inner buffer.")
        
    else:
        print("Destination is outside the safe inner buffer.")

@app.route('/checkConnection', methods=['GET'])
def check_connection():
    if vehicle:
        return jsonify({"vehicle": "connected"}), 200
    else:
        return jsonify({"error": "Vehicle connection not established."}), 500

@app.route('/getPosition', methods=['GET'])
def getPosition():
    if vehicle:
        try:
            current_position = vehicle.get_current_position()
            return jsonify({"current_position": current_position})
        except Exception as e:
            return jsonify({"error": str(e)}), 500
    else:
        return jsonify({"error": "Vehicle connection not established."}), 500

@app.route('/navigateBoatToPoint', methods=['GET'])
def navigateBoatToPoint():
    destination = request.body.get('destination')
    try:
        navigate(destination)
        return jsonify({"message": "Navigation to point initiated."}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


if __name__ == "__main__":
    vehicle = VehicleConnection()
    if RUN_API:
        app.run(debug=True)
    else:
        navigate(None)