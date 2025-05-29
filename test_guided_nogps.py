#!/usr/bin/env python3
import time
from pymavlink import mavutil

def main():
    # 1) Connect to SITL on UDP 14550 and wait for heartbeat
    mav = mavutil.mavlink_connection("udp:127.0.0.1:15550")
    mav.wait_heartbeat()
    print("✅ Heartbeat received. Waiting for GUIDED_NOGPS...")

    # 2) Loop until we see GUIDED_NOGPS
    while True:
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb is None:
            continue

        # Map the custom_mode to a human string
        modes = mav.mode_mapping()
        mode_str = modes.get(hb.custom_mode, f"UNKNOWN({hb.custom_mode})")
        print(f"[{time.strftime('%X')}] Current mode: {mode_str}")

        if mode_str == "GUIDED_NOGPS":
            print("🎉 Detected GUIDED_NOGPS! Test successful.")
            break

        time.sleep(0.5)

if __name__ == "__main__":
    main()
