#!/usr/bin/env python3
from pymavlink import mavutil
import time
import math

# Global start time for time_boot_ms calculation.
start_time = time.monotonic()

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles (in radians) to quaternion.
    Returns a list: [w, x, y, z]
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y, z]

def connect_to_sitl():
    connection_string = "udp:127.0.0.1:14550"
    print("Connecting to SITL via UDP on", connection_string)
    master = mavutil.mavlink_connection(connection_string)
    master.wait_heartbeat()
    print("Connected. Heartbeat from system (system %u, component %u)" %
          (master.target_system, master.target_component))
    return master

def send_attitude_target(master, roll=0.0, pitch=0.0, yaw=0.0, thrust=0.5):
    """
    Send a SET_ATTITUDE_TARGET message.
    Uses type_mask=0 so that all fields (including thrust) are active.
    Thrust of ~0.5 is assumed to maintain altitude.
    """
    q = quaternion_from_euler(roll, pitch, yaw)
    type_mask = 0  # do not ignore any parameters.
    # Use elapsed time since the script started (in milliseconds).
    time_boot_ms = int((time.monotonic() - start_time) * 1000)
    
    master.mav.set_attitude_target_send(
        time_boot_ms,
        master.target_system,
        master.target_component,
        type_mask,
        q,
        0, 0, 0,  # body roll rate, pitch rate, yaw rate (not used)
        thrust
    )
    print(f"Attitude cmd: roll=0, pitch={pitch:.3f}, yaw={yaw:.3f}, thrust={thrust:.3f}")

def get_gps_position(master, timeout=1):
    """
    Wait for a GLOBAL_POSITION_INT message and extract latitude, longitude, altitude.
    Returns values in degrees (lat, lon) and meters (alt).
    """
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
    if msg is None:
        return None, None, None
    lat = msg.lat / 1e7
    lon = msg.lon / 1e7
    alt = msg.alt / 1000.0  # Convert mm to m.
    return lat, lon, alt

def quantize_position(lat, lon):
    """
    Quantize the position to simulate low precision (approx. 500 m resolution).
    Latitude is quantized in steps of ~500 m (~0.0045°).
    Longitude is quantized based on the cosine of the latitude.
    """
    lat_step = 500 / 111000.0   # about 0.0045°
    cos_lat = math.cos(math.radians(lat))
    lon_step = 500 / (111000.0 * cos_lat) if cos_lat != 0 else 0.005
    q_lat = round(lat / lat_step) * lat_step
    q_lon = round(lon / lon_step) * lon_step
    return q_lat, q_lon

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Compute the great-circle distance between two latitude/longitude points.
    Returns distance in meters.
    """
    R = 6371000.0  # Earth radius in meters.
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c

def compute_bearing(lat1, lon1, lat2, lon2):
    """
    Compute the bearing in radians from point 1 (lat1, lon1) to point 2 (lat2, lon2).
    Bearing is normalized to 0–2π.
    """
    dLon = math.radians(lon2 - lon1)
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    x = math.sin(dLon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dLon)
    bearing = math.atan2(x, y)
    if bearing < 0:
        bearing += 2 * math.pi
    return bearing

def main():
    master = connect_to_sitl()
    
    # Assume the aircraft is already airborne.
    print("Switching to GUIDED_NOGPS mode")
    master.set_mode("GUIDED_NOGPS")
    
    # Define the flight plan (box) as a series of waypoints.
    # Order: Home point, then corner 2, corner 3, corner 4.
    waypoints = [
        (-35.363262, 149.165237),      # Home (starting point)
        (-35.3187709, 149.1570670),
        (-35.3110088, 149.2113521),
        (-35.3552028, 149.2211129)
    ]
    
    # Start at the first waypoint (Home).
    current_target_index = 1   # Next target is the second point.
    threshold_distance = 1000  # In meters; threshold distance for waypoint switch.
    commanded_yaw = 0.0        # The commanded yaw (in radians).
    
    last_guidance_update = time.time()
    
    print("Entering main control loop (press Ctrl+C to exit)...")
    try:
        while True:
            # Command forward motion using a slight forward pitch.
            forward_pitch_deg = 5
            forward_pitch_rad = -math.radians(forward_pitch_deg)  # negative pitch = forward motion.
            send_attitude_target(master, roll=0.0, pitch=forward_pitch_rad, yaw=commanded_yaw, thrust=0.5)
            
            current_time = time.time()
            # Update guidance every 3 seconds.
            if current_time - last_guidance_update >= 3.0:
                last_guidance_update = current_time
                
                gps_lat, gps_lon, gps_alt = get_gps_position(master, timeout=1)
                if gps_lat is None:
                    print("No GPS data received; skipping guidance update.")
                else:
                    # Quantize position to simulate 500-m resolution.
                    q_lat, q_lon = quantize_position(gps_lat, gps_lon)
                    print(f"Raw GPS: ({gps_lat:.6f}, {gps_lon:.6f}), Quantized: ({q_lat:.6f}, {q_lon:.6f}), Alt: {gps_alt:.1f} m")
                    
                    # Get current target waypoint.
                    target_lat, target_lon = waypoints[current_target_index]
                    
                    # Compute distance to the current target.
                    distance_to_wp = haversine_distance(q_lat, q_lon, target_lat, target_lon)
                    print(f"Distance to waypoint {current_target_index}: {distance_to_wp:.1f} m")
                    
                    # If within threshold, switch to the next waypoint.
                    if distance_to_wp < threshold_distance:
                        print(f"Reached waypoint {current_target_index}. Switching to next waypoint.")
                        current_target_index = (current_target_index + 1) % len(waypoints)
                        target_lat, target_lon = waypoints[current_target_index]
                    
                    # Compute and update commanded yaw toward current target.
                    commanded_yaw = compute_bearing(q_lat, q_lon, target_lat, target_lon)
                    print(f"Updated commanded yaw: {math.degrees(commanded_yaw):.1f}° toward waypoint {current_target_index}")
                    
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("Control loop interrupted. Exiting...")

if __name__ == '__main__':
    main()
