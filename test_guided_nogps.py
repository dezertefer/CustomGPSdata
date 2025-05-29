#!/usr/bin/env python3
import time
from pymavlink import mavutil

# ─── EDIT THIS LIST TO WATCH FOR MODES YOU WANT TO TEST ──────────────
TARGET_MODES = {
    "STABILIZE",
    "ALT_HOLD",
    "LOITER",
    "GUIDED",
    "GUIDED_NOGPS",
    "AUTO",
    "RTL",
}
# ──────────────────────────────────────────────────────────────────────

def main():
    mav = mavutil.mavlink_connection("udp:127.0.0.1:15550")
    mav.wait_heartbeat()
    print("✅ Heartbeat OK. Watching for modes:", ", ".join(sorted(TARGET_MODES)))
    last_mode = None

    while True:
        hb = mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if not hb:
            continue

        # get human‐readable mode string
        mode_str = mavutil.mode_string_v10(hb)

        # only print on change to reduce spam
        if mode_str != last_mode:
            print(f"[{time.strftime('%X')}] Mode changed → {mode_str}")
            if mode_str in TARGET_MODES:
                print(f"✔️  Detected target mode: {mode_str}")
            last_mode = mode_str

        time.sleep(0.2)

if __name__ == "__main__":
    main()
