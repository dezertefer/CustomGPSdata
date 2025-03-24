import time
from pymavlink import mavutil

def main():
    # Connect to Ardupilot (adjust the connection string and port as needed)
    master = mavutil.mavlink_connection('udpout:127.0.0.1:14550')
    master.wait_heartbeat()
    print("Connected to system (system %u component %u)" % (master.target_system, master.target_component))

    # Define initial GPS data (using GPS_RAW_INT message format)
    # Note: lat and lon are in degrees * 1e7, altitude in mm
    lat = int(47.397742 * 1e7)   # example latitude
    lon = int(8.545594 * 1e7)    # example longitude
    alt = int(500 * 1000)        # example altitude: 500 meters
    eph = 100                    # GPS horizontal dilution of precision
    epv = 100                    # GPS vertical dilution of precision
    vel = 0                    # GPS ground speed (cm/s)
    cog = 0                    # course over ground (degrees * 100)
    satellites_visible = 10      # number of satellites

    # Send simulated GPS messages continuously
    while True:
        # The first parameter is the timestamp in milliseconds
        master.mav.gps_raw_int_send(
            int(time.time() * 1000),  # time stamp (ms)
            3,                       # fix_type (3 for 3D fix)
            lat,
            lon,
            alt,
            eph,
            epv,
            vel,
            cog,
            satellites_visible
        )
        print("Simulated GPS message sent")
        time.sleep(1)  # adjust frequency as needed

if __name__ == '__main__':
    main()
