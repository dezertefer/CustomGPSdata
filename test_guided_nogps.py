#!/usr/bin/env python3
import time
from pymavlink import mavutil

def main():
    mav = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    mav.wait_heartbeat()
    print("✅ Heartbeat OK, waiting for GUIDED_NOGPS…")

    while True:
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if not hb:
            continue

        mode_str = mavutil.mode_string_v10(hb)
        print(f"[{time.strftime('%X')}] Mode: {mode_str}")

        if mode_str == "GUIDED_NOGPS":
            print("🎉 Detected GUIDED_NOGPS!")
            break

        time.sleep(0.5)

if __name__ == "__main__":
    main()
