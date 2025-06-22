#!/usr/bin/env python3
# gps.py  – attitude-only guidance while in GUIDED_NOGPS
#
# Adds “turn-rate limiting”:  the yaw set-point is slewed toward the
# desired bearing at ≤ TURN_RATE_DEG °/s.  
# A 3ʳᵈ Web-Socket “/rate” (port 8767) lets the browser change that value
# at run-time by sending  {"turn_rate": <deg_per_sec>}.

import asyncio, json, math, time, websockets
from   pymavlink import mavutil

# ───────── configuration ─────────
SITL_URL          = "udp:127.0.0.1:15550"
STREAM_HZ         = 2
FWD_PITCH_RAD     = -math.radians(5)    # ≈ –5 °
ARRIVAL_RADIUS_M_DEFAULT = 300         # ← rename & keep old value
arrival_radius_m         = ARRIVAL_RADIUS_M_DEFAULT
HEX_PREFIX        = "Mode("             # ignore opaque hex names
# ──────────────────────────────────

mav                           = None          # MAVLink connection
current_pos, target_pos       = None, None    # last messages from sockets
TURN_RATE_DEG_S               = 10.0          # default 10 °/s
turn_rate_rad_s               = math.radians(TURN_RATE_DEG_S)
cmd_yaw                        = 0.0          # yaw we are currently commanding
t0                             = time.monotonic()

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
    q    = q_from_euler(0,pitch,yaw)
    t_ms = int((time.monotonic()-t0)*1000)
    mav.mav.set_attitude_target_send(t_ms, mav.target_system, mav.target_component,
                                     0, q, 0,0,0, thrust)

def haversine(lat1,lon1,lat2,lon2):
    R=6_371_000
    φ1,φ2=map(math.radians,(lat1,lat2))
    dφ,dλ=map(math.radians,(lat2-lat1, lon2-lon1))
    a=math.sin(dφ/2)**2+math.cos(φ1)*math.cos(φ2)*math.sin(dλ/2)**2
    return 2*R*math.asin(math.sqrt(a))

def bearing(lat1,lon1,lat2,lon2):
    φ1,φ2=map(math.radians,(lat1,lat2)); dλ=math.radians(lon2-lon1)
    x=math.sin(dλ)*math.cos(φ2)
    y=math.cos(φ1)*math.sin(φ2)-math.sin(φ1)*math.cos(φ2)*math.cos(dλ)
    b=math.atan2(x,y)
    return b if b>=0 else b+2*math.pi

def ang_diff(target, current):
    """Signed shortest-way angular difference (rad)."""
    d = (target - current + math.pi) % (2*math.pi) - math.pi
    return d

# ───────── Web-Socket handlers ─────────
async def handle_current(ws):
    global current_pos
    print("🛰  /current connected")
    try:
        async for msg in ws:
            try: current_pos = json.loads(msg)
            except json.JSONDecodeError: print("❌ bad /current JSON")
    finally: print("🛰  /current disconnected")

async def handle_target(ws):
    global target_pos
    print("🎯 /target connected")
    try:
        async for msg in ws:
            try:
                target_pos = json.loads(msg)
                print(f"🎯 new target {target_pos}")
            except json.JSONDecodeError: print("❌ bad /target JSON")
    finally: print("🎯 /target disconnected")

async def handle_rate(ws):
    global TURN_RATE_DEG_S, turn_rate_rad_s
    print("⤵️  /rate connected")
    try:
        async for msg in ws:
            try:
                data = json.loads(msg)
                if 'turn_rate' in data:
                    TURN_RATE_DEG_S = float(data['turn_rate'])
                    turn_rate_rad_s = math.radians(TURN_RATE_DEG_S)
                    print(f"🔧 turn-rate now {TURN_RATE_DEG_S:.1f} °/s")
            except Exception as e:
                print("❌ bad /rate message:", e)
    finally: print("⤵️  /rate disconnected")

async def handle_radius(ws):
    global arrival_radius_m
    print("➰ /radius connected")
    try:
        async for msg in ws:
            try:
                data = json.loads(msg)
                if 'radius' in data:
                    arrival_radius_m = float(data['radius'])
                    print(f"🔧 arrival-radius now {arrival_radius_m:.1f} m")
            except Exception as e:
                print("❌ bad /radius message:", e)
    finally:
        print("➰ /radius disconnected")

# ───────── control helpers ─────────
def hb_fcu():
    """Return next FCU (component-1) heartbeat, or None if none pending."""
    while True:
        msg = mav.recv_match(type='HEARTBEAT', blocking=False)
        if not msg:                   return None
        if msg.get_srcComponent()==1: return msg

def is_guided_nogps(hb):
    mode = mavutil.mode_string_v10(hb)
    return mode=="GUIDED_NOGPS" or mode.startswith(HEX_PREFIX)

async def guided_loop():
    """Runs while FCU stays in GUIDED_NOGPS."""
    global cmd_yaw
    arrived=False
    dt=0.2
    while True:
        hb = hb_fcu()
        if hb and not is_guided_nogps(hb):
            print(f"🚫 Mode changed → {mavutil.mode_string_v10(hb)}")
            return                                  # stop commanding

        desired_yaw = cmd_yaw       # default: keep previous
        pitch        = 0.0

        if current_pos and target_pos:
            dist = haversine(current_pos['latitude'],current_pos['longitude'],
                             target_pos ['latitude'],target_pos ['longitude'])
            if dist > arrival_radius_m:
                pitch       = FWD_PITCH_RAD
                desired_yaw = bearing(current_pos['latitude'],current_pos['longitude'],
                                      target_pos ['latitude'],target_pos ['longitude'])
            elif not arrived:
                print(f"✅ arrived (≤{ARRIVAL_THRESH_M} m) – level")
                arrived = True
        else:
            arrived = False

        # --- rate-limit yaw -------------------------------------------------
        dy   = ang_diff(desired_yaw, cmd_yaw)
        step = turn_rate_rad_s * dt
        if abs(dy) > step:
            cmd_yaw += math.copysign(step, dy)
            cmd_yaw %= 2*math.pi
        else:
            cmd_yaw = desired_yaw

        send_attitude(pitch, cmd_yaw)
        await asyncio.sleep(dt)

# ───────── main ─────────
async def main():
    global mav
    # 1) Web-Socket servers
    await websockets.serve(handle_current,"0.0.0.0",8765)
    await websockets.serve(handle_target ,"0.0.0.0",8766)
    await websockets.serve(handle_rate   ,"0.0.0.0",8767)
    await websockets.serve(handle_radius ,"0.0.0.0",8768)   # ← NEW
    print("🌐 sockets – /current:8765  /target:8766  /rate:8767  /radius:8768")

    # 2) MAVLink
    print(f"→ Connecting to {SITL_URL}")
    mav = mavutil.mavlink_connection(SITL_URL)
    mav.wait_heartbeat(); print("✅ Heartbeat received")

    mav.mav.request_data_stream_send(mav.target_system,mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, STREAM_HZ, 1)
    print(f"→ Requested DATA_STREAM_ALL @ {STREAM_HZ} Hz")

    # 3) state-machine
    while True:
        print("⏳ Awaiting GUIDED_NOGPS …")
        while True:
            hb = hb_fcu()
            if hb and is_guided_nogps(hb):
                print("▶ GUIDED_NOGPS detected – control loop starts")
                break
            await asyncio.sleep(0.25)
        await guided_loop()

if __name__=="__main__":
    try: asyncio.run(main())
    except KeyboardInterrupt: print("\n⏹ terminated")
