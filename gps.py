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

def send_attitude_target(master, roll=0.0, pitch=0.0, yaw=0.0):
    """
    Send SET_ATTITUDE_TARGET message with a type mask that ignores throttle.
    - roll, pitch, yaw are in radians.
    - Thrust is ignored so the autopilot holds altitude.
    """
    # Convert Euler angles to quaternion
    q = quaternion_from_euler(roll, pitch, yaw)
    
    # Set type_mask bits:
    # Bit 0: ignore body roll rate
    # Bit 1: ignore body pitch rate
    # Bit 2: ignore body yaw rate
    # Bit 3: ignore thrust (so autopilot controls throttle)
    type_mask = 0b00001111  # decimal 15

    # Calculate time_boot_ms as elapsed time since script start (in milliseconds)
    time_boot_ms = int((time.monotonic() - start_time) * 1000)
    
    master.mav.set_attitude_target_send(
        time_boot_ms,                    # time_boot_ms in milliseconds
        master.target_system,            # target system
        master.target_component,         # target component
        type_mask,                       # type mask (ignore rates and thrust)
        q,                               # Quaternion [w, x, y, z]
        0, 0, 0,                         # Body roll rate, pitch rate, yaw rate (ignored)
        0.0                              # Thrust (ignored due to type_mask)
    )
    print(f"Sent attitude target: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f} (thrust ignored)")

def move_drone_attitude(master, forward=True):
    """
    Commands a slight pitch to move the drone forward or backward.
    For forward motion, pitch is set to a negative angle; for backward, positive.
    """
    angle_deg = 5.0            # 5 degrees pitch
    angle_rad = math.radians(angle_deg)
    # Negative pitch for forward, positive for backward
    pitch = -angle_rad if forward else angle_rad
    direction = "forward" if forward else "backward"
    print(f"Commanding {direction} movement with pitch angle: {pitch:.3f} radians")
    send_attitude_target(master, roll=0.0, pitch=pitch, yaw=0.0)

def main():
    master = connect_to_sitl()

    # Loop: alternate forward and backward movement.
    for i in range(10):
        print(f"Cycle {i+1}: Moving forward")
        move_drone_attitude(master, forward=True)
        time.sleep(1)
        print(f"Cycle {i+1}: Moving backward")
        move_drone_attitude(master, forward=False)
        time.sleep(1)

    # Disarm the drone.
    print("Disarming the drone")
    master.arducopter_disarm()
    master.close()

if __name__ == '__main__':
    main()
