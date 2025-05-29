#!/usr/bin/env python3
import time
from pymavlink import mavutil

def main():
    # 1) Connect and log
    sitl_addr = "udp:127.0.0.1:15550"
    print(f"→ Connecting to SITL on {sitl_addr}")
    mav = mavutil.mavlink_connection(sitl_addr)
    print("→ Waiting for heartbeat…")
    mav.wait_heartbeat()
    print(f"✅ Connected (sys={mav.target_system} comp={mav.target_component})")

    # 2) Ask the autopilot to stream all data at 2 Hz
    mav.mav.request_data_stream_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        2,    # 2 Hz
        1     # start
    )
    print("→ Requested DATA_STREAM_ALL @2 Hz")

    # 3) Poll for HEARTBEATs and print any mode changes
    last_mode = None
    while True:
        hb = mav.recv_match(type="HEARTBEAT", blocking=False)
        if hb:
            mode_str = mavutil.mode_string_v10(hb)
            if mode_str != last_mode:
                print(f"[{time.strftime('%X')}] Mode → {mode_str}")
                last_mode = mode_str
        time.sleep(0.2)

if __name__ == "__main__":
    main()
