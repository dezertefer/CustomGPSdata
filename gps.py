#!/usr/bin/env python3
import asyncio
import websockets
import json
import time
import math
from pymavlink import mavutil

# globals for async handlers
current_pos = None
target_pos  = None
mav_master  = None
start_time  = time.monotonic()

async def handle_current(ws, path):
    global current_pos
    print("🛰 Current‐pos client connected")
    try:
        async for msg in ws:
            current_pos = json.loads(msg)
            print(f"[{time.strftime('%X')}] Current ← {current_pos}")
    except websockets.exceptions.ConnectionClosed:
        print("🛰 Current‐pos client disconnected")

async def handle_target(ws, path):
    global target_pos
    print("🎯 Target‐pos client connected")
    try:
        async for msg in ws:
            target_pos = json.loads(msg)
            print(f"[{time.strftime('%X')}] Target ← {target_pos}")
    except websockets.exceptions.ConnectionClosed:
        print("🎯 Target‐pos client disconnected")

def quaternion_from_euler(roll, pitch, yaw):
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cr, sr = math.cos(roll/2),    math.sin(roll/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return [w, x, y, z]

def send_attitude_target(roll, pitch, yaw, thrust=0.5):
    """
    Send SET_ATTITUDE_TARGET to mav_master.
    small forward pitch → motion; thrust=0.5 for hover.
    """
    q = quaternion_from_euler(roll, pitch, yaw)
    type_mask = 0  # use all fields
    t_ms = int((time.monotonic() - start_time) * 1000)
    mav_master.mav.set_attitude_target_send(
        t_ms,
        mav_master.target_system,
        mav_master.target_component,
        type_mask,
        q, 0, 0, 0,
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

async def control_loop():
    """
    - Every 0.2s: send a small forward‐pitch attitude target using current commanded_yaw.
    - Every 3s: recompute commanded_yaw = bearing(current→target).
    """
    commanded_yaw = 0.0
    last_update   = time.time()
    while True:
        # always send forward‐motion command
        pitch = -math.radians(5)  # negative = forward
        send_attitude_target(0.0, pitch, commanded_yaw)

        # recompute yaw every 3s
        if time.time() - last_update >= 3.0:
            last_update = time.time()
            if current_pos and target_pos:
                dlat = target_pos['lat'] - current_pos['lat']
                dlon = target_pos['lon'] - current_pos['lon']
                dist = math.hypot(dlat, dlon) * 111000  # rough meters
                new_yaw = compute_bearing(
                    current_pos['lat'], current_pos['lon'],
                    target_pos ['lat'], target_pos ['lon']
                )
                print(f"[{time.strftime('%X')}] Δ={dist:.0f} m → yaw {math.degrees(new_yaw):.1f}°")
                commanded_yaw = new_yaw

        await asyncio.sleep(0.2)

async def main():
    global mav_master
    # 1) connect to SITL
    mav_master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    mav_master.wait_heartbeat()
    print("🎮 Connected to SITL (GUIDED_NOGPS)")
    mav_master.set_mode("GUIDED_NOGPS")

    # 2) start websocket servers
    srv1 = websockets.serve(handle_current, 'localhost', 8765)
    srv2 = websockets.serve(handle_target,  'localhost', 8766)
    await asyncio.gather(srv1, srv2)
    print("🚀 WS servers on 8765 (current) & 8766 (target)")

    # 3) run control loop forever
    await control_loop()

if __name__ == '__main__':
    asyncio.run(main())
