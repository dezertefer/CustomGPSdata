import asyncio, websockets, json, time, math, traceback
from pymavlink import mavutil

current_pos = None
target_pos  = None
mav_master  = None
start_time  = time.monotonic()

async def handle_current(ws, path):
    global current_pos
    print("🛰 Current‐pos handler ready")
    try:
        async for msg in ws:
            try:
                current_pos = json.loads(msg)
                print(f"[{time.strftime('%X')}] Current ← {current_pos}")
            except json.JSONDecodeError as e:
                print("❌ JSON error in current:", e)
    except Exception:
        print("‼️ Exception in handle_current:")
        traceback.print_exc()
    finally:
        print("🛰 Current handler exit")

async def handle_target(ws, path):
    global target_pos
    print("🎯 Target‐pos handler ready")
    try:
        async for msg in ws:
            try:
                target_pos = json.loads(msg)
                print(f"[{time.strftime('%X')}] Target ← {target_pos}")
            except json.JSONDecodeError as e:
                print("❌ JSON error in target:", e)
    except Exception:
        print("‼️ Exception in handle_target:")
        traceback.print_exc()
    finally:
        print("🎯 Target handler exit")

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


async def control_loop():
    commanded_yaw   = 0.0
    last_update     = time.time()
    arrival_thresh  = 300    # metres – adjust for your noise level
    arrived         = False

    while True:
        if arrived:
            # keep hovering: zero pitch, same yaw, thrust ≈ hover
            send_attitude_target(0, 0, commanded_yaw, thrust=0.5)
            await asyncio.sleep(0.5)
            continue

        # normal forward-motion command (5° forward pitch)
        send_attitude_target(0, -math.radians(5), commanded_yaw)

        # guidance update every 3 s
        if time.time() - last_update >= 3.0 and current_pos and target_pos:
            last_update = time.time()

            lat1, lon1 = current_pos['lat'], current_pos['lon']
            lat2, lon2 = target_pos ['lat'], target_pos ['lon']
            dlat, dlon = lat2 - lat1, lon2 - lon1
            dist       = math.hypot(dlat, dlon) * 111000  # rough metres

            print(f"[{time.strftime('%X')}] distance ≈ {dist:.0f} m")

            if dist < arrival_thresh:
                print(f"🎉 Arrived (< {arrival_thresh} m). Hovering.")
                arrived = True
                continue

            # update heading toward target
            commanded_yaw = compute_bearing(lat1, lon1, lat2, lon2)
            print(f"→ new heading {math.degrees(commanded_yaw):.1f}°")

        await asyncio.sleep(0.2)

async def main():
    global mav_master
    mav_master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
    mav_master.wait_heartbeat()
    mav_master.set_mode("GUIDED_NOGPS")
    await websockets.serve(handle_current, 'localhost', 8765)
    await websockets.serve(handle_target,  'localhost', 8766)
    print("Servers up on 8765 & 8766")
    await control_loop()

if __name__ == '__main__':
    asyncio.run(main())
