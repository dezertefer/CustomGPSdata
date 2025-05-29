#!/usr/bin/env python3
import asyncio
import websockets
import json
import time
import math
import traceback
from pymavlink import mavutil

# ---------------------------------------------------------------------
#  WebSocket state
# ---------------------------------------------------------------------
current_pos = None          # {'lat': …, 'lon': …, 'declination': …}
target_pos  = None          # {'lat': …, 'lon': …, 'declination': …}
target_ws   = None          # websocket connection (to send ACK)

# ---------------------------------------------------------------------
#  MAVLink helpers
# ---------------------------------------------------------------------
START_TIME = time.monotonic()
mav = None                  # will hold our MAVLink connection

def quaternion_from_euler(r, p, y):
    cy, sy = math.cos(y/2), math.sin(y/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cr, sr = math.cos(r/2), math.sin(r/2)
    return [
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ]

def send_attitude_target(pitch_rad, yaw_rad, thrust=0.5):
    q = quaternion_from_euler(0, pitch_rad, yaw_rad)
    t_ms = int((time.monotonic() - START_TIME) * 1000)
    mav.mav.set_attitude_target_send(
        t_ms,
        mav.target_system,
        mav.target_component,
        0,      # type_mask: use roll/pitch/yaw & thrust
        q, 0, 0, 0,
        thrust
    )

def haversine(lat1, lon1, lat2, lon2):
    R = 6_371_000
    φ1, φ2 = map(math.radians, (lat1, lat2))
    dφ  = math.radians(lat2 - lat1)
    dλ  = math.radians(lon2 - lon1)
    a = math.sin(dφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(dλ/2)**2
    return 2 * R * math.asin(math.sqrt(a))

def bearing(lat1, lon1, lat2, lon2):
    dλ = math.radians(lon2 - lon1)
    φ1, φ2 = map(math.radians, (lat1, lat2))
    x = math.sin(dλ) * math.cos(φ2)
    y = math.cos(φ1) * math.sin(φ2) - math.sin(φ1) * math.cos(φ2) * math.cos(dλ)
    brng = math.atan2(x, y)
    return brng if brng >= 0 else brng + 2*math.pi

# ---------------------------------------------------------------------
#  WebSocket handlers
# ---------------------------------------------------------------------
async def handle_current(ws):
    """Receives {"latitude":.., "longitude":.., "declination":..} from simulator."""
    global current_pos
    print("🛰  /current connected")
    try:
        async for msg in ws:
            try:
                data = json.loads(msg)
                current_pos = {
                    'lat':         data['latitude'],
                    'lon':         data['longitude'],
                    'declination': data.get('declination')
                }
            except (json.JSONDecodeError, KeyError):
                print("❌ bad JSON from /current; expected latitude & longitude")
    except Exception:
        traceback.print_exc()
    finally:
        print("🛰  /current disconnected")

async def handle_target(ws):
    """
    Receives {"latitude":.., "longitude":.., "declination":..} for the next waypoint.
    Sends back {"reached": true} on the same socket when within threshold.
    """
    global target_pos, target_ws
    target_ws = ws
    print("🎯 /target connected")
    try:
        async for msg in ws:
            try:
                data = json.loads(msg)
                target_pos = {
                    'lat':         data['latitude'],
                    'lon':         data['longitude'],
                    'declination': data.get('declination')
                }
                print("🎯 new target:", target_pos)
            except (json.JSONDecodeError, KeyError):
                print("❌ bad JSON from /target; expected latitude & longitude")
    except Exception:
        traceback.print_exc()
    finally:
        target_ws = None
        print("🎯 /target disconnected")

# ---------------------------------------------------------------------
#  Main control loop
# ---------------------------------------------------------------------
async def control_loop():
    arrival_thresh = 300         # metres “close enough”
    forward_pitch  = -math.radians(30)   # ~2.6 m/s
    commanded_yaw  = 0.0
    last_update    = time.time()

    while True:
        # 1) Check for mode change
        hb = mav.recv_match(type='HEARTBEAT', blocking=False)
        if hb:
            mode_str = mav.mode_mapping().get(hb.custom_mode)
            if mode_str != "GUIDED_NOGPS":
                print(f"⚠️ Mode changed to {mode_str}, exiting control loop.")
                return  # exit back to main

        # 2) Send attitude every 0.2 s
        send_attitude_target(forward_pitch, commanded_yaw)
        await asyncio.sleep(0.2)

        # 3) Guidance logic every 3 s
        if time.time() - last_update < 3.0 or not (current_pos and target_pos):
            continue
        last_update = time.time()

        dist = haversine(
            current_pos['lat'], current_pos['lon'],
            target_pos ['lat'], target_pos ['lon']
        )
        print(f"[{time.strftime('%X')}] dist≈{dist:.0f} m")

        if dist <= arrival_thresh:
            print("✅ reached target (≤300 m)")
            if target_ws:
                await target_ws.send(json.dumps({"reached": True}))
            forward_pitch = 0  # hover until new target
        else:
            # recompute heading
            commanded_yaw = bearing(
                current_pos['lat'], current_pos['lon'],
                target_pos ['lat'], target_pos ['lon']
            )
            forward_pitch = -math.radians(5)

# ---------------------------------------------------------------------
#  Program entry
# ---------------------------------------------------------------------
async def main():
    global mav

    # 1) Connect to SITL and wait for heartbeat
    mav = mavutil.mavlink_connection("udp:127.0.0.1:15550")
    mav.wait_heartbeat()
    print("✅ Heartbeat received. Waiting for GUIDED_NOGPS mode…")

    # 2) Block until GUIDED_NOGPS
    while True:
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb:
            mode_str = mav.mode_mapping().get(hb.custom_mode)
            if mode_str == "GUIDED_NOGPS":
                print("🔄 Detected GUIDED_NOGPS — starting control servers.")
                break
        await asyncio.sleep(0.5)

    # 3) Start WebSocket servers
    await websockets.serve(handle_current, "0.0.0.0", 8765)
    await websockets.serve(handle_target,  "0.0.0.0", 8766)
    print("🌐 WebSocket servers running on 8765 (/current) & 8766 (/target)")

    # 4) Enter the control loop
    await control_loop()

    print("👋 Control loop exited; shutting down.")

if __name__ == "__main__":
    asyncio.run(main())
