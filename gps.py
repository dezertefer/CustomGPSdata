#!/usr/bin/env python3
# drone_control_server.py
import asyncio, websockets, json, time, math, traceback, subprocess, os, sys
from pymavlink import mavutil

# ---------------------------------------------------------------------
#  WebSocket state
# ---------------------------------------------------------------------
current_pos = None          # {"lat": …, "lon": …}
target_pos  = None          # {"lat": …, "lon": …}
target_ws   = None          # websocket connection (to send ACK)

# ---------------------------------------------------------------------
#  MAVLink helpers
# ---------------------------------------------------------------------
START_TIME = time.monotonic()
mav = None                  # global connection

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
    # type_mask = 0 → use roll/pitch/yaw & thrust
    mav.mav.set_attitude_target_send(t_ms,
                                     mav.target_system,
                                     mav.target_component,
                                     0,
                                     q, 0,0,0,
                                     thrust)

def haversine(lat1, lon1, lat2, lon2):
    R = 6_371_000
    φ1, φ2 = map(math.radians, (lat1, lat2))
    dφ  = math.radians(lat2 - lat1)
    dλ  = math.radians(lon2 - lon1)
    a = math.sin(dφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(dλ/2)**2
    return 2*R*math.asin(math.sqrt(a))

def bearing(lat1, lon1, lat2, lon2):
    dλ = math.radians(lon2 - lon1)
    φ1, φ2 = map(math.radians, (lat1, lat2))
    x = math.sin(dλ)*math.cos(φ2)
    y = math.cos(φ1)*math.sin(φ2) - math.sin(φ1)*math.cos(φ2)*math.cos(dλ)
    brng = math.atan2(x, y)
    return brng if brng >= 0 else brng + 2*math.pi

# ---------------------------------------------------------------------
#  WebSocket handlers
# ---------------------------------------------------------------------
async def handle_current(ws):
    """Receives {"lat": .., "lon": ..} from simulator."""
    global current_pos
    print("🛰  /current connected")
    try:
        async for msg in ws:
            try:
                current_pos = json.loads(msg)
            except json.JSONDecodeError:
                print("❌ bad JSON from current feed")
    except Exception:
        traceback.print_exc()
    finally:
        print("🛰  /current disconnected")

async def handle_target(ws):
    """
    Receives {"lat": .., "lon": ..} for the next waypoint.
    The control loop sends back {"reached": true} on the
    *same* websocket when arrival threshold is met.
    """
    global target_pos, target_ws
    target_ws = ws
    print("🎯 /target connected")
    try:
        async for msg in ws:
            try:
                target_pos = json.loads(msg)
                print("🎯 new target:", target_pos)
            except json.JSONDecodeError:
                print("❌ bad JSON from target feed")
    except Exception:
        traceback.print_exc()
    finally:
        target_ws = None
        print("🎯 /target disconnected")

# ---------------------------------------------------------------------
#  Main control loop
# ---------------------------------------------------------------------
async def control_loop():
    arrival_thresh = 300      # metres “close enough”
    forward_pitch  = -math.radians(30)   # 5° nose-down → ~2.6 m/s
    commanded_yaw  = 0.0
    last_update    = time.time()

    while True:
        # send attitude every 0.2 s
        send_attitude_target(forward_pitch, commanded_yaw)
        await asyncio.sleep(0.2)

        # guidance every 3 s
        if time.time() - last_update < 3.0 or not (current_pos and target_pos):
            continue
        last_update = time.time()

        dist = haversine(current_pos['lat'], current_pos['lon'],
                         target_pos ['lat'], target_pos ['lon'])
        print(f"[{time.strftime('%X')}] dist≈{dist:.0f} m")

        if dist <= arrival_thresh:
            print("✅ reached target (≤300 m)")
            # notify target client so it can send the next corner
            if target_ws:
                await target_ws.send(json.dumps({"reached": True}))
            # keep holding present yaw until a NEW target arrives
            forward_pitch = 0        # level off until new target
            continue

        # still en route → compute yaw to target
        commanded_yaw  = bearing(current_pos['lat'], current_pos['lon'],
                                 target_pos ['lat'], target_pos ['lon'])
        forward_pitch  = -math.radians(5)

# ---------------------------------------------------------------------
#  Program entry
# ---------------------------------------------------------------------
async def main():
    global mav, target_ws, current_pos, target_pos

    # 1) Connect to SITL and wait for heartbeat
    mav = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    mav.wait_heartbeat()
    print("✅ MAVLink heartbeat received. Waiting for GUIDED_NOGPS mode...")

    # 2) Watch for manual mode switch to GUIDED_NOGPS
    while True:
        hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if hb:
            mode_str = mav.mode_mapping().get(hb.custom_mode)
            if mode_str == "GUIDED_NOGPS":
                print("🔄 Detected GUIDED_NOGPS — launching helpers.")
                break
        await asyncio.sleep(0.5)

    # 3) Launch helper scripts from the same directory
    script_dir   = os.path.dirname(os.path.abspath(__file__))
    sim_script   = os.path.join(script_dir, "current_position_simulator_square.py")
    tgt_script   = os.path.join(script_dir, "target_sender.py")

    # Use the same Python interpreter
    python_exec = sys.executable

    sim_proc = subprocess.Popen([python_exec, sim_script])
    tgt_proc = subprocess.Popen([python_exec, tgt_script])
    print(f"👟 Launched simulator (PID {sim_proc.pid}) and target sender (PID {tgt_proc.pid}).")

    # 4) Start both WebSocket servers
    await websockets.serve(handle_current, "0.0.0.0", 8765)
    await websockets.serve(handle_target,  "0.0.0.0", 8766)
    print("🌐 WebSocket servers running on 8765 (/current) & 8766 (/target)")

    # 5) Enter the control loop
    await control_loop()

if __name__ == "__main__":
    asyncio.run(main())
