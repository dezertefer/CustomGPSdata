from pymavlink import mavutil
import time

# Connect to the SITL
connection_string = "tcp:127.0.0.1:5760"
master = mavutil.mavlink_connection(connection_string)
master.wait_heartbeat()
print("Connected to SITL on 127.0.0.1:5760")

# Set the mode to GUIDED
print("Setting mode to GUIDED")
master.set_mode(mavutil.mavlink.MAV_MODE_GUIDED_ARMED)

# Arm the drone
print("Arming the drone")
master.arducopter_arm()

# Wait for the drone to arm
time.sleep(2)

# Function to send the desired movement command
def move_drone(forward=True):
    # Define forward/backward velocity
    velocity = 1.0 if forward else -1.0
    print(f"Moving {'forward' if forward else 'backward'} with velocity {velocity}")

    # Send the velocity command (move in the X direction)
    master.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms
        master.target_system,  # target_system
        master.target_component,  # target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # coordinate frame
        0b0000111111001000,  # position/control flags (ignore altitude, velocity control)
        0,  # x position (m)
        0,  # y position (m)
        0,  # z position (m)
        velocity,  # x velocity (m/s)
        0,  # y velocity (m/s)
        0,  # z velocity (m/s)
        0,  # acceleration in x (m/s^2)
        0,  # acceleration in y (m/s^2)
        0,  # acceleration in z (m/s^2)
        0,  # yaw rate (rad/s)
        0  # yaw (rad)
    )

# Move the drone forward and backward in a loop
for _ in range(10):  # 10 cycles of forward and backward
    move_drone(forward=True)
    time.sleep(1)
    move_drone(forward=False)
    time.sleep(1)

# Disarm the drone
print("Disarming the drone")
master.arducopter_disarm()

# Close the connection
master.close()
