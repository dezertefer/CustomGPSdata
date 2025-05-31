#!/usr/bin/env python3
# gps.py  –  attitude-only guidance while the FCU is in GUIDED_NOGPS
#
# ─────────────────────────────────────────────────────────────────────────
# • Web-Socket servers (/current → 8765, /target → 8766) start first and
#   remain up, so a browser or other client can connect at any time.
# • Once the autopilot enters GUIDED_NOGPS the script begins pitching the
#   aircraft forward until it is “close enough” to the target. When either
#   feed is missing, or the distance falls below ARRIVAL_THRESH_M, the nose
#   levels out automatically.
# • If the autopilot leaves GUIDED_NOGPS the loop levels the aircraft,
#   stops sending SET_ATTITUDE_TARGET and waits for the mode to return.
# ─────────────────────────────────────────────────────────────────────────

import asyncio, json, math, time, websockets
from   pymavlink import mavutil

# ─────────────── user-tunable settings ────────────────
SITL_URL         = "udp:127.0.0.1:15550"   # where mavlink-router exposes the SITL
STREAM_HZ        = 2                       # data-stream request rate
FWD_PITCH_RAD    = -math.radians(5)        # ≈ –5 °
ARRIVAL_THRESH_M = 300                     # “close enough” radius
HEX_PREFIX       = "Mode("                 # ignore opaque hex mode labels
# ──────────────────────────────────────────────────────

mav           = None
current_pos   = None      # {"latitude": .., "longitude": .., ...}
target_pos    = None
start_time    = time.monotonic()

# ───────── MAVLink helpers ─────────
def q_from_euler(r, p, y):
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]

def send_attitude(pitch, yaw, thrust=0.5):
    """Send one SET_ATTITUDE_TARGET frame (roll fixed at 0 rad)."""
    q    = q_from_euler(0, pitch, yaw)
    t_ms = int((time.monotonic() - start_time) * 1_000)
    mav.mav.set_attitude_target_send(
        t_ms, mav.target_system, mav.target_component,
        0,                     # type-mask 0 → use roll/pitch/yaw + thrust
        q,
        0, 0, 0,               # body-rates (rad/s) — unused
        thrust
    )

def haversine(lat1, lon1, lat2, lon2):
    R = 6_371_000
    φ1, φ2 = map(math.radians, (lat1, lat2))
    dφ, dλ = map(math.radians, (lat2 - lat1, lon2 - lon1))
    a = math.sin(dφ / 2) ** 2 + math.cos(φ1) * math.cos(φ2) * math.sin(dλ / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))

def bearing(lat1, lon1, lat2, lon2):
    φ1, φ2 = map(math.radians, (lat1, lat2))
    dλ     = math.radians(lon2 - lon1)
    x = math.sin(dλ) * math.cos(φ2)
    y = math.cos(φ1) * math.sin(φ2) - math.sin(φ1) * math.cos(φ2) * math.cos(dλ)
    b = math.atan2(x, y)
    return b if b >= 0 else b + 2 * math.pi

# ───────── Web-Socket handlers ─────────
async def handle_current(ws):
    global current_pos
    print("🛰  /current connected")
    try:
        async for msg in ws:
            try:
                current_pos = json.loads(msg)
            except json.JSONDecodeError:
                print("❌ bad /current JSON")
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
            except json.JSONDecodeError:
                print("❌ bad /target JSON")
    finally:
        print("🎯 /target disconnected")

# ───────── mode helpers ─────────
def mode_is_guided_nogps(hb) -> bool:
    """
    True  → FCU really is in GUIDED_NOGPS
    False → any other *actual* mode (plain GUIDED, Loiter, ...)
    Hex “Mode(0x…​)” strings are ignored because SITL sometimes emits them
    between valid string names.
    """
    name = mavutil.mode_string_v10(hb)
    if name == "GUIDED_NOGPS":
        return True
    if name.startswith(HEX_PREFIX):        # transient/no-info label – ignore
        return True
    return False

# ───────── control loop ─────────
async def guided_loop():
    arrived = False
    while True:
        # 1) Abort immediately if FCU left GUIDED_NOGPS
        hb = mav.recv_match(type='HEARTBEAT', blocking=False)
        if hb and not mode_is_guided_nogps(hb):
            print(f"🚫 Mode changed → {mavutil.mode_string_v10(hb)} (pausing)")
            send_attitude(0, 0)            # level the aircraft
            return

        # 2) Compute guidance
        yaw   = 0
        pitch = 0
        if current_pos and target_pos:
            dist = haversine(
                current_pos['latitude'],  current_pos['longitude'],
                target_pos ['latitude'],  target_pos ['longitude']
            )
            if dist <= ARRIVAL_THRESH_M:
                if not arrived:
                    print(f"✅ arrived (≤{ARRIVAL_THRESH_M} m) – holding level")
                    arrived = True
            else:
                arrived = False
                yaw   = bearing(
                    current_pos['latitude'],  current_pos['longitude'],
                    target_pos ['latitude'],  target_pos ['longitude']
                )
                pitch = FWD_PITCH_RAD
        else:
            arrived = False                # wait for both feeds

        # 3) Send attitude every 200 ms
        send_attitude(pitch, yaw)
        await asyncio.sleep(0.2)

# ───────── program entry ─────────
async def main():
    global mav

    # 1) Launch Web-Socket servers *first*
    await websockets.serve(handle_current, "0.0.0.0", 8765)
    await websockets.serve(handle_target,  "0.0.0.0", 8766)
    print("🌐 Web-Socket servers on 8765 (/current) & 8766 (/target)")

    # 2) Connect to SITL
    print(f"→ Connecting to SITL on {SITL_URL}")
    mav = mavutil.mavlink_connection(SITL_URL)
    mav.wait_heartbeat()
    print("✅ Heartbeat received")

    # 3) Ask for a modest data-stream
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, STREAM_HZ, 1)
    print(f"→ Requested DATA_STREAM_ALL @ {STREAM_HZ} Hz")

    # 4) Wait → run → wait … cycle
    while True:
        print("⏳ Awaiting GUIDED_NOGPS …")
        while True:
            hb = mav.recv_match(type='HEARTBEAT', blocking=False)
            if hb and mode_is_guided_nogps(hb):
                print("▶ GUIDED_NOGPS detected – starting control loop")
                break
            await asyncio.sleep(0.25)      # keeps the event-loop responsive
        await guided_loop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n⏹ terminated")
