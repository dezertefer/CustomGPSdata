#!/usr/bin/env python3
# gps.py – attitude-only guidance while in GUIDED_NOGPS
#
# • Web-socket servers stay up permanently
# • Live-tunable parameters
#      ↘️ forward-pitch   ws://<pi>:8769  {"pitch_deg": 1-45}
#      ⤵️ turn-rate       ws://<pi>:8767  {"rate_deg_s": N}
#      ➰ arrival-radius  ws://<pi>:8768  {"radius_m":   N}
# • /current  (8765)  – sim-or-VOR position feed
#   /target   (8766)  – single waypoint feed

import asyncio, json, math, time, websockets
from   pymavlink import mavutil

# ───────── static config ─────────
SITL_URL       = "udp:127.0.0.1:15550"
STREAM_HZ      = 2                              # request FCU streams @ 2 Hz
HEX_PREFIX     = "Mode("                       # ignore opaque mode names

TURN_RATE_DEG_DEFAULT = 20                     # °/s
ARRIVAL_RADIUS_DEFAULT = 300                   # metres
PITCH_DEG_DEFAULT      = 5                     # nose-down (positive)

# ───────── live-tunable variables ─────────
turn_rate_deg_s  = TURN_RATE_DEG_DEFAULT
arrival_radius_m = ARRIVAL_RADIUS_DEFAULT
forward_pitch_rad = -math.radians(PITCH_DEG_DEFAULT)

# ───────── run-time state ─────────
mav            = None
current_pos    = None
target_pos     = None
commanded_yaw  = 0.0
t0             = time.monotonic()

# ───────── MAV helpers ─────────
def q_from_euler(r,p,y):
    cy,sy = math.cos(y/2), math.sin(y/2)
    cp,sp = math.cos(p/2), math.sin(p/2)
    cr,sr = math.cos(r/2), math.sin(r/2)
    return [cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy]

def send_attitude(pitch,yaw,thrust=0.5):
    q = q_from_euler(0, pitch, yaw)
    t_ms = int((time.monotonic()-t0)*1000)
    mav.mav.set_attitude_target_send(t_ms,
                                     mav.target_system,
                                     mav.target_component,
                                     0, q, 0,0,0, thrust)

def haversine(lat1,lon1,lat2,lon2):
    R=6_371_000
    φ1,φ2 = map(math.radians,(lat1,lat2))
    dφ,dλ = map(math.radians,(lat2-lat1, lon2-lon1))
    a = math.sin(dφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(dλ/2)**2
    return 2*R*math.asin(math.sqrt(a))

def bearing(lat1,lon1,lat2,lon2):
    φ1,φ2 = map(math.radians,(lat1,lat2)); dλ = math.radians(lon2-lon1)
    x = math.sin(dλ)*math.cos(φ2)
    y = math.cos(φ1)*math.sin(φ2) - math.sin(φ1)*math.cos(φ2)*math.cos(dλ)
    b = math.atan2(x,y)
    return b if b>=0 else b + 2*math.pi

# ───────── websocket handlers ─────────
async def handle_current(ws):
    global current_pos
    print("🛰  /current connected")
    try:
        async for msg in ws:
            try: current_pos = json.loads(msg)
            except: print("❌ bad /current JSON")
    finally:
        print("🛰  /current disconnected")

async def handle_target(ws):
    global target_pos
    print("🎯 /target connected")
    try:
        async for msg in ws:
            try:
                target_pos = json.loads(msg)
                print(f"🎯 new target {target_pos}")
            except: print("❌ bad /target JSON")
    finally:
        print("🎯 /target disconnected")

async def handle_rate(ws):
    global turn_rate_deg_s
    print("⤵️  /rate connected")
    try:
        async for msg in ws:
            try:
                deg = float(json.loads(msg).get("rate_deg_s", TURN_RATE_DEG_DEFAULT))
                turn_rate_deg_s = max(1.0, min(180.0, deg))
                print(f"🔧 turn-rate now {turn_rate_deg_s:.1f} °/s")
            except Exception as e:
                print("❌ bad /rate JSON:", e)
    finally:
        print("⤵️  /rate disconnected")

async def handle_radius(ws):
    global arrival_radius_m
    print("➰ /radius connected")
    try:
        async for msg in ws:
            try:
                r = float(json.loads(msg).get("radius_m", ARRIVAL_RADIUS_DEFAULT))
                arrival_radius_m = max(1.0, r)
                print(f"🔧 arrival-radius now {arrival_radius_m:.1f} m")
            except Exception as e:
                print("❌ bad /radius JSON:", e)
    finally:
        print("➰ /radius disconnected")

async def handle_pitch(ws):
    global forward_pitch_rad
    print("↘️  /pitch connected")
    try:
        async for msg in ws:
            try:
                deg = float(json.loads(msg).get("pitch_deg", PITCH_DEG_DEFAULT))
                deg = max(1, min(45, deg))
                forward_pitch_rad = -math.radians(deg)
                print(f"🔧 forward-pitch now {deg:.1f}°")
            except Exception as e:
                print("❌ bad /pitch JSON:", e)
    finally:
        print("↘️  /pitch disconnected")

# ───────── control helpers ─────────
def hb_fcu():
    """return next FCU heartbeat if queued"""
    while True:
        m = mav.recv_match(type='HEARTBEAT', blocking=False)
        if not m:         return None
        if m.get_srcComponent()==1: return m

def in_guided_nogps(hb):
    m = mavutil.mode_string_v10(hb)
    return m=="GUIDED_NOGPS" or m.startswith(HEX_PREFIX)

# ───────── main control loop ─────────
async def guided_loop():
    """
    Runs while the FCU stays in GUIDED_NOGPS.

    * No attitude commands are sent until BOTH /current and /target
      have arrived.
    * The very first real bearing we compute becomes the initial
      commanded_yaw, so we keep whatever heading we had.
    """
    global commanded_yaw
    arrived   = False
    last_time = time.monotonic()

    while True:
        # ① mode guard -------------------------------------------------
        hb = hb_fcu()
        if hb and not in_guided_nogps(hb):
            print(f"🚫 Mode changed → {mavutil.mode_string_v10(hb)}")
            return                                   # leave loop

        # ② need both feeds before we do anything ----------------------
        if not (current_pos and target_pos):
            await asyncio.sleep(0.1)
            continue

        # ③ compute distance & bearing --------------------------------
        dist = haversine(current_pos['latitude'],current_pos['longitude'],
                         target_pos ['latitude'],target_pos ['longitude'])
        tgt_bearing = bearing(current_pos['latitude'],current_pos['longitude'],
                              target_pos ['latitude'],target_pos ['longitude'])

        # ④ first-time initialisation of commanded_yaw ----------------
        if commanded_yaw is None:
            commanded_yaw = tgt_bearing        # keep current nose-angle
            # NB: we *don’t* send an attitude yet – next loop will

        # ⑤ arrival test ----------------------------------------------
        if dist <= arrival_radius_m:
            if not arrived:
                print(f"✅ arrived (≤{arrival_radius_m:.1f} m) – level")
                arrived = True
            pitch_cmd = 0
        else:
            arrived   = False
            pitch_cmd = forward_pitch_rad

        # ⑥ turn-rate limiter -----------------------------------------
        now = time.monotonic()
        dt  = now - last_time
        last_time = now

        max_step = math.radians(turn_rate_deg_s) * dt
        diff = (tgt_bearing - commanded_yaw + math.pi) % (2*math.pi) - math.pi
        if abs(diff) > max_step:
            diff = math.copysign(max_step, diff)
        commanded_yaw = (commanded_yaw + diff + 2*math.pi) % (2*math.pi)

        # ⑦ finally, send the attitude --------------------------------
        send_attitude(pitch_cmd, commanded_yaw)
        await asyncio.sleep(0.1)

# ───────── program entry ─────────
async def main():
    global mav
    # websockets first
    await websockets.serve(handle_current, "0.0.0.0", 8765)
    await websockets.serve(handle_target , "0.0.0.0", 8766)
    await websockets.serve(handle_rate   , "0.0.0.0", 8767)
    await websockets.serve(handle_radius , "0.0.0.0", 8768)
    await websockets.serve(handle_pitch  , "0.0.0.0", 8769)
    print("🌐 sockets → 8765/current  8766/target  8767/rate "
          "8768/radius  8769/pitch")

    # MAVLink
    print(f"→ Connecting to {SITL_URL}")
    mav = mavutil.mavlink_connection(SITL_URL)
    mav.wait_heartbeat();   print("✅ heartbeat")

    mav.mav.request_data_stream_send(mav.target_system,mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, STREAM_HZ, 1)

    # wait-run-wait loop
    while True:
        print("⏳ waiting GUIDED_NOGPS …")
        while True:
            hb = hb_fcu()
            if hb and in_guided_nogps(hb):
                print("▶ GUIDED_NOGPS detected – loop start")
                break
            await asyncio.sleep(0.25)
        await guided_loop()

if __name__=="__main__":
    try: asyncio.run(main())
    except KeyboardInterrupt: print("\n⏹ terminated")
