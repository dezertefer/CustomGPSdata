#!/usr/bin/env python3
# gps.py  –  attitude-only guidance while in GUIDED_NOGPS
#  * forward pitch only when BOTH feeds present AND not yet at the target
#  * when “arrived”, pitch goes back to 0 ° until a fresh target arrives

import asyncio, json, math, time, websockets
from   pymavlink import mavutil

# ───────── configuration ─────────
SITL_URL         = "udp:127.0.0.1:15550"
STREAM_HZ        = 2
FWD_PITCH_RAD    = -math.radians(5)       # ~ nose-down 5 °
ARRIVAL_THRESH_M = 300
HEX_PREFIX       = "Mode("                # opaque labels to ignore
# ──────────────────────────────────

mav           = None
current_pos   = None
target_pos    = None
start_time    = time.monotonic()

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
    t_ms = int((time.monotonic()-start_time)*1000)
    mav.mav.set_attitude_target_send(t_ms,
        mav.target_system, mav.target_component,
        0, q, 0,0,0, thrust)

def haversine(lat1,lon1,lat2,lon2):
    R = 6_371_000
    φ1,φ2 = map(math.radians,(lat1,lat2))
    dφ = math.radians(lat2-lat1)
    dλ = math.radians(lon2-lon1)
    a  = math.sin(dφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(dλ/2)**2
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
            try:
                target_pos=json.loads(msg)
                print(f"🎯 new target {target_pos}")
            except: print("❌ bad /target JSON")
    finally:
        print("🎯  /target disconnected")

# ───────── control loop ─────────
def still_guided(hb):
    name=mavutil.mode_string_v10(hb)
    if name=="GUIDED_NOGPS":                return True
    if name.startswith(HEX_PREFIX):         return True   # ignore hex blobs
    return False

async def guided_loop():
    """Runs while we remain in GUIDED_NOGPS; exits on any real mode change."""
    arrived = False          # remember if we’ve reached the current target
    while True:
        # check mode
        hb = mav.recv_match(type='HEARTBEAT',blocking=False)
        if hb and not still_guided(hb):
            print(f"🚫 Mode changed → {mavutil.mode_string_v10(hb)}")
            send_attitude(0,0)              # level out
            return                          # leave loop

        yaw=0; pitch=0
        if current_pos and target_pos:
            dist = haversine(current_pos['latitude'],current_pos['longitude'],
                             target_pos ['latitude'],target_pos ['longitude'])
            if dist <= ARRIVAL_THRESH_M:
                if not arrived:
                    print(f"✅ arrived (≤{ARRIVAL_THRESH_M} m) – holding level")
                    arrived=True
            else:
                arrived=False
                yaw   = bearing(current_pos['latitude'],current_pos['longitude'],
                                target_pos ['latitude'],target_pos ['longitude'])
                pitch = FWD_PITCH_RAD
        else:
            arrived=False                  # missing data → not arrived

        send_attitude(pitch,yaw)
        await asyncio.sleep(0.2)

# ───────── main ─────────
async def main():
    global mav
    print(f"→ Connecting to SITL on {SITL_URL}")
    mav = mavutil.mavlink_connection(SITL_URL)
    mav.wait_heartbeat()
    print("✅ Heartbeat received")

    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, STREAM_HZ, 1)
    print(f"→ Requested DATA_STREAM_ALL @ {STREAM_HZ} Hz")

    # WebSocket servers
    await websockets.serve(handle_current,"0.0.0.0",8765)
    await websockets.serve(handle_target ,"0.0.0.0",8766)
    print("🌐 WebSocket servers on 8765 (/current) & 8766 (/target)")

    # wait → run → wait cycle
    while True:
        print("⏳ Awaiting GUIDED_NOGPS …")
        while True:
            hb=mav.recv_match(type='HEARTBEAT',blocking=True,timeout=1)
            if hb and mavutil.mode_string_v10(hb)=="GUIDED_NOGPS":
                print("▶ GUIDED_NOGPS detected – starting control loop")
                break
        await guided_loop()

if __name__=="__main__":
    try: asyncio.run(main())
    except KeyboardInterrupt: print("\n⏹ terminated")
