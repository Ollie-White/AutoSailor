from pymavlink import mavutil
import json
import time

#Load configuration from external file
with open('environment-variables.json', 'r') as file:
    config = json.load(file)

USE_SIMULATOR = config['use_simulator']
SIMULATOR_START = config['starting_coordinates']

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
            if not mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("Arming vehicle...")
                self.connection.arducopter_arm()
                time.sleep(1)
            else:
                print("Vehicle already armed!")
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