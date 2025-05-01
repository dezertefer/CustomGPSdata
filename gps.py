#!/usr/bin/env python3
import asyncio
import websockets
import json
import time
import math
from pymavlink import mavutil

# Globals for sharing state
current_pos = None
target_pos  = None
mav_master  = None
start_time  = time.monotonic()

#
# —— WebSocket Handlers ——
#
async def handle_current(websocket, path):
    global current_pos
    print("🛰 Current‐pos client connected")
    try:
        async for msg in websocket:
            try:
                data = json.loads(msg)
                current_pos = data
                print(f"[{time.strftime('%X')}] Current ← {current_pos}")
            except Exception as e:
                print("❌ Invalid JSON in current handler:", e)
    except websockets.exceptions.ConnectionClosed:
        print("🛰 Current‐pos client disconnected")

async def handle_target(websocket, path):
    global target_pos
    print("🎯 Target‐pos client connected")
    try:
        async for msg in websocket:
            try:
                data = json.loads(msg)
                target_pos = data
                print(f"[{time.strftime('%X')}] Target ← {target_pos}")
            except Exception as e:
                print("❌ Invalid JSON in target handler:", e)
    except websockets.exceptions.ConnectionClosed:
        print("🎯 Target‐pos client disconnected")

#
# —— MAVLink Helpers ——
#
def quaternion_from_euler(roll, pitch, yaw):
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cr, sr = math.cos(roll/2),    math.sin(roll/2)
    return [
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy
    ]

def send_attitude_target(roll, pitch, yaw, thrust=0.5):
    t_ms = int((time.monotonic() - start_time) * 1000)
    q    = quaternion_from_euler(roll, pitch, yaw)
    mav_master.mav.set_attitude_target_send(
        t_ms,
        mav_master.target_system,
        mav_master.target_component,
        0,      # type_mask = 0 → use all fields (including thrust)
        q, 0,0,0,
        thrust
    )
    print(f"→ Att cmd: pitch={pitch:.3f}, yaw={math.degrees(yaw):.1f}°, thrust={thrust}")

def compute_bearing(lat1, lon1, lat2, lon2):
    dLon = math.radians(lon2 - lon1)
    φ1, φ2 = math.radians(lat1), math.radians(lat2)
    x = math.sin(dLon)*math.cos(φ2)
    y = math.cos(φ1)*math.sin(φ2) - math.sin(φ1)*math.cos(φ2)*math.cos(dLon)
    bearing = math.atan2(x, y)
    return bearing if bearing >= 0 else bearing + 2*math.pi

#
# —— Control Loop —— 
#
async def control_loop():
    commanded_yaw = 0.0
    last_update   = time.time()
    while True:
        # 1) Always send a forward‐pitch attitude so the drone moves.
        send_attitude_target(0.0, -math.radians(5), commanded_yaw)

        # 2) Every 3s, recompute heading if we have both positions.
        if time.time() - last_update >= 3.0:
            last_update = time.time()
            if current_pos and target_pos:
                lat1, lon1 = current_pos['lat'], current_pos['lon']
                lat2, lon2 = target_pos ['lat'], target_pos ['lon']
                # rough distance for debug
                dlat, dlon = lat2-lat1, lon2-lon1
                dist = math.hypot(dlat, dlon)*111000
                yaw  = compute_bearing(lat1, lon1, lat2, lon2)
                print(f"[{time.strftime('%X')}] Δ={dist:.0f} m → yaw {math.degrees(yaw):.1f}°")
                commanded_yaw = yaw
        await asyncio.sleep(0.2)

#
# —— Main Setup —— 
#
async def main():
    global mav_master
    # 1) Connect to SITL
    mav_master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    mav_master.wait_heartbeat()
    print("🎮 Connected to SITL, switching to GUIDED_NOGPS")
    mav_master.set_mode("GUIDED_NOGPS")

    # 2) Start both WebSocket servers
    server1 = await websockets.serve(handle_current, "localhost", 8765)
    server2 = await websockets.serve(handle_target,  "localhost", 8766)
    print("🚀 WS servers running on ports 8765 (current) & 8766 (target)")

    # 3) Kick off the control loop
    await control_loop()

if __name__ == '__main__':
    asyncio.run(main())
