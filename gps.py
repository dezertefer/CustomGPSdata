#!/usr/bin/env python3
# gps.py  –  control loop that runs only while in GUIDED_NOGPS
# (opaque “Mode(0x…)” labels are now ignored)

import asyncio, json, math, time, traceback
from pymavlink import mavutil
import websockets

# ────────────── configuration ──────────────
SITL_URL         = "udp:127.0.0.1:15550"
STREAM_HZ        = 2
PITCH_FWD_RAD    = -math.radians(5)      # forward tilt when ready
ARRIVAL_THRESH_M = 300
# ───────────────────────────────────────────

mav = None
current_pos = None
target_pos  = None
START_TIME  = time.monotonic()

# ───────── MAV helpers ─────────
def q_from_euler(r,p,y):
    cy,sy = math.cos(y/2), math.sin(y/2)
    cp,sp = math.cos(p/2), math.sin(p/2)
    cr,sr = math.cos(r/2), math.sin(r/2)
    return [cr*cp*cy+sr*sp*sy,
            sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy]

def send_attitude(pitch,yaw,thrust=0.5):
    q = q_from_euler(0,pitch,yaw)
    t_ms=int((time.monotonic()-START_TIME)*1000)
    mav.mav.set_attitude_target_send(t_ms,
        mav.target_system,mav.target_component,0,q,0,0,0, thrust)

def haversine(lat1,lon1,lat2,lon2):
    R=6_371_000
    φ1,φ2 = map(math.radians,(lat1,lat2))
    dφ=math.radians(lat2-lat1); dλ=math.radians(lon2-lon1)
    a=math.sin(dφ/2)**2+math.cos(φ1)*math.cos(φ2)*math.sin(dλ/2)**2
    return 2*R*math.asin(math.sqrt(a))

def bearing(lat1,lon1,lat2,lon2):
    dλ=math.radians(lon2-lon1); φ1,φ2=map(math.radians,(lat1,lat2))
    x=math.sin(dλ)*math.cos(φ2)
    y=math.cos(φ1)*math.sin(φ2)-math.sin(φ1)*math.cos(φ2)*math.cos(dλ)
    b=math.atan2(x,y)
    return b if b>=0 else b+2*math.pi

# ───────── WebSocket handlers ─────────
async def handle_current(ws):
    global current_pos
    print("🛰  /current connected")
    try:
        async for msg in ws:
            try: current_pos=json.loads(msg)
            except: print("❌ bad /current JSON")
    finally:
        print("🛰  /current disconnected")

async def handle_target(ws):
    global target_pos
    print("🎯  /target connected")
    try:
        async for msg in ws:
            try: target_pos=json.loads(msg)
            except: print("❌ bad /target JSON")
    finally:
        print("🎯  /target disconnected")

# ───────── control loop ─────────
IGNORED_PREFIX = "Mode("       # treat these hex-labels as “same mode”

def still_guided(hb):
    name = mavutil.mode_string_v10(hb)
    if name == "GUIDED_NOGPS":          # definitely still guided
        return True
    if name.startswith(IGNORED_PREFIX): # opaque/hex → ignore
        return True
    return False                        # a real, named different mode

async def guided_loop():
    """Runs while we remain in GUIDED_NOGPS."""
    last_report=""
    while True:
        hb=mav.recv_match(type='HEARTBEAT',blocking=False)
        if hb and not still_guided(hb):
            print(f"🚫 Mode changed to {mavutil.mode_string_v10(hb)}")
            send_attitude(0,0)          # level before exiting
            return

        ready = current_pos and target_pos
        pitch = PITCH_FWD_RAD if ready else 0
        yaw   = 0
        if ready:
            dist=haversine(current_pos['latitude'],current_pos['longitude'],
                           target_pos ['latitude'],target_pos ['longitude'])
            yaw = bearing(current_pos['latitude'],current_pos['longitude'],
                          target_pos ['latitude'],target_pos ['longitude'])
            if dist<=ARRIVAL_THRESH_M and last_report!="arrived":
                print(f"✅ reached target (≤{ARRIVAL_THRESH_M} m)")
                last_report="arrived"
            elif dist>ARRIVAL_THRESH_M and last_report!="en-route":
                print(f"↗ dist≈{dist:.0f} m")
                last_report="en-route"
        send_attitude(pitch,yaw)
        await asyncio.sleep(0.2)

# ───────── main ─────────
async def main():
    global mav
    print(f"→ Connecting to SITL on {SITL_URL}")
    mav=mavutil.mavlink_connection(SITL_URL)
    mav.wait_heartbeat()
    print("✅ Heartbeat received")

    mav.mav.request_data_stream_send(
        mav.target_system,mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, STREAM_HZ,1)
    print(f"→ Requested DATA_STREAM_ALL @ {STREAM_HZ} Hz")

    await websockets.serve(handle_current,"0.0.0.0",8765)
    await websockets.serve(handle_target ,"0.0.0.0",8766)
    print("🌐 WebSocket servers on 8765 (/current) & 8766 (/target)")

    # wait-run-wait cycle
    while True:
        print("⏳ Awaiting GUIDED_NOGPS …")
        while True:
            hb=mav.recv_match(type='HEARTBEAT',blocking=True,timeout=1)
            if hb and mavutil.mode_string_v10(hb)=="GUIDED_NOGPS":
                print("▶ GUIDED_NOGPS detected")
                break
        await guided_loop()

if __name__=="__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n⏹  terminated")
