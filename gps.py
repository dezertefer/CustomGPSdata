#!/usr/bin/env python3
import asyncio
import websockets
import json
import time
import math
import traceback
from pymavlink import mavutil

# ── CONFIG ────────────────────────────────────────────────────────────────
MAVLINK_URI    = "udp:127.0.0.1:15550"   # the port mavlink-router forwards to
DATASTREAM_HZ  = 2                       # how often to ask for all streams

# ── WebSocket state ───────────────────────────────────────────────────────
current_pos = None    # {'lat':…, 'lon':…, 'declination':…}
target_pos  = None    # {'lat':…, 'lon':…, 'declination':…}
target_ws   = None    # so we can ACK back {"reached":true}

# ── MAVLink helpers ───────────────────────────────────────────────────────
START_TIME = time.monotonic()
mav = None

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
        0,      # type_mask
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
    x = math.sin(dλ)*math.cos(φ2)
    y = math.cos(φ1)*math.sin(φ2) - math.sin(φ1)*math.cos(φ2)*math.cos(dλ)
    brng = math.atan2(x, y)
    return brng if brng >= 0 else brng + 2*math.pi

# ── WebSocket handlers ────────────────────────────────────────────────────
async def handle_current(ws):
    print("🛰  /current connected")
    global current_pos
    try:
        async for msg in ws:
            try:
                data = json.loads(msg)
                current_pos = {
                    'lat': data['latitude'],
                    'lon': data['longitude'],
                    'declination': data.get('declination')
                }
            except (json.JSONDecodeError, KeyError):
                print("❌ bad JSON on /current; need latitude & longitude")
    except Exception:
        traceback.print_exc()
    finally:
        print("🛰  /current disconnected")

async def handle_target(ws):
    print("🎯 /target connected")
    global target_pos, target_ws
    target_ws = ws
    try:
        async for msg in ws:
            try:
                data = json.loads(msg)
                target_pos = {
                    'lat': data['latitude'],
                    'lon': data['longitude'],
                    'declination': data.get('declination')
                }
                print("🎯 new target:", target_pos)
            except (json.JSONDecodeError, KeyError):
                print("❌ bad JSON on /target; need latitude & longitude")
    except Exception:
        traceback.print_exc()
    finally:
        target_ws = None
        print("🎯 /target disconnected")

# ── Control loop ──────────────────────────────────────────────────────────
async def control_loop():
    arrival_thresh = 300       # metres
    forward_pitch  = -math.radians(30)
    commanded_yaw  = 0.0
    last_update    = time.time()

    while True:
        # 1) Watch for mode flips
        hb = mav.recv_match(type='HEARTBEAT', blocking=False)
        if hb:
            print(f"[{time.strftime('%X')}] raw custom_mode → {hb.custom_mode!r}")
            mode_str = mav.mode_mapping().get(hb.custom_mode)
            print(f"[{time.strftime('%X')}] decoded mode      → {mode_str}")
            if mode_str != "GUIDED_NOGPS":
                print("⚠️  Mode left GUIDED_NOGPS; exiting loop.")
                return

        # 2) Send attitude every 0.2 s
        send_attitude_target(forward_pitch, commanded_yaw)
        await asyncio.sleep(0.2)

        # 3) Guidance every 3 s
        if time.time() - last_update < 3.0 or not (current_pos and target_pos):
            continue
        last_update = time.time()

        dist = haversine(
            current_pos['lat'], current_pos['lon'],
            target_pos ['lat'], target_pos ['lon']
        )
        print(f"[{time.strftime('%X')}] dist≈{dist:.0f} m")

        if dist <= arrival_thresh:
            print("✅ reached target")
            if target_ws:
                await target_ws.send(json.dumps({"reached": True}))
            forward_pitch = 0.0
        else:
            commanded_yaw = bearing(
                current_pos['lat'], current_pos['lon'],
                target_pos ['lat'], target_pos ['lon']
            )
            forward_pitch = -math.radians(5)

# ── Entry point ───────────────────────────────────────────────────────────
async def main():
    global mav

    # 1) connect to SITL via mavlink-router port
    mav = mavutil.mavlink_connection(MAVLINK_URI)
    mav.wait_heartbeat()
    print(f"✅ Heartbeat received on {MAVLINK_URI}")

    # 2) request full data-stream so mode_mapping() gets populated
    mav.mav.request_data_stream_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        DATASTREAM_HZ, 1
    )
    print(f"→ Requested DATA_STREAM_ALL @ {DATASTREAM_HZ} Hz")

    # 3) spin up WS servers immediately
    print("🌐 Starting WebSocket servers…")
    curr_srv = await websockets.serve(handle_current, "0.0.0.0", 8765)
    tgt_srv  = await websockets.serve(handle_target,  "0.0.0.0", 8766)
    print("✅ WebSocket servers up on 8765 (/current) & 8766 (/target)")

    # 4) wait until user flips into GUIDED_NOGPS
    print("⏳ Awaiting GUIDED_NOGPS mode…")
    while True:
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb:
            mode_str = mav.mode_mapping().get(hb.custom_mode)
            print(f"[{time.strftime('%X')}] startup mode → {mode_str}")
            if mode_str == "GUIDED_NOGPS":
                print("🔄 Entering control loop.")
                break
        await asyncio.sleep(0.5)

    # 5) run control loop (exits when mode changes)
    await control_loop()

    # 6) cleanup
    print("👋 Shutting down WebSocket servers.")
    curr_srv.close();  tgt_srv.close()
    await curr_srv.wait_closed();  await tgt_srv.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())
