from pymavlink import mavutil
import time
import math

# Global start time for time_boot_ms calculation
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
    Send SET_ATTITUDE_TARGET message.
    This version uses a type_mask of 0 (all parameters active) so that the autopilot
    can manage throttle (with a default thrust value that maintains hover).
    """
    # Convert Euler angles to quaternion
    q = quaternion_from_euler(roll, pitch, yaw)
    
    # type_mask: 0 means no fields are ignored.
    type_mask = 0b00000000  # decimal 0
    
    # Calculate time_boot_ms as elapsed time since script start (in milliseconds)
    time_boot_ms = int((time.monotonic() - start_time) * 1000)
    
    master.mav.set_attitude_target_send(
        time_boot_ms,                    # time_boot_ms in milliseconds
        master.target_system,            # target system
        master.target_component,         # target component
        type_mask,                       # type mask (all fields used)
        q,                               # Quaternion [w, x, y, z]
        0, 0, 0,                         # Body roll rate, pitch rate, yaw rate (not used)
        thrust                           # Thrust value to hold hover (adjust if needed)
    )
    print(f"Sent attitude target: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}, thrust={thrust:.3f}")

def move_forward(master, current_yaw, duration=5, pitch_deg=5):
    """
    Commands forward motion for a given duration.
    A slight negative pitch (in radians) is used to induce forward motion.
    """
    pitch_rad = -math.radians(pitch_deg)  # negative for forward motion
    end_time = time.time() + duration
    while time.time() < end_time:
        send_attitude_target(master, roll=0.0, pitch=pitch_rad, yaw=current_yaw, thrust=0.5)
        time.sleep(0.2)

def turn_in_place(master, target_yaw, duration=2):
    """
    Commands a turn in place (hover without forward pitch) to reach a new yaw angle.
    """
    end_time = time.time() + duration
    while time.time() < end_time:
        # No forward pitch: pitch=0
        send_attitude_target(master, roll=0.0, pitch=0.0, yaw=target_yaw, thrust=0.5)
        time.sleep(0.2)

def main():
    master = connect_to_sitl()

    # Set mode to GUIDED and arm the drone.
    print("Setting mode to GUIDED")
    master.set_mode(mavutil.mavlink.MAV_MODE_GUIDED_ARMED)
    print("Arming the drone")
    master.arducopter_arm()
    time.sleep(2)  # wait for arming

    # Start with an initial heading (yaw) of 0 radians.
    current_yaw = 0.0

    # Fly a square: 4 sides
    for i in range(4):
        print(f"Side {i+1}: Moving forward for 5 seconds at yaw {math.degrees(current_yaw):.1f}°")
        move_forward(master, current_yaw, duration=5, pitch_deg=5)
        # Calculate new yaw for a 90° clockwise turn.
        # For ArduPilot, yaw is in degrees with 0° as North; increasing yaw typically turns clockwise.
        current_yaw += math.radians(90)
        # Wrap-around if needed.
        current_yaw = current_yaw % (2 * math.pi)
        print(f"Side {i+1}: Turning in place to yaw {math.degrees(current_yaw):.1f}°")
        turn_in_place(master, current_yaw, duration=2)

    # After completing the square, hover for a few seconds.
    print("Hovering...")
    for _ in range(10):
        send_attitude_target(master, roll=0.0, pitch=0.0, yaw=current_yaw, thrust=0.5)
        time.sleep(0.2)

    # Disarm and close the connection.
    print("Disarming the drone")
    master.arducopter_disarm()
    master.close()

if __name__ == '__main__':
    main()
