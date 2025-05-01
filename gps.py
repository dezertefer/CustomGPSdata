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

def quaternion_from_euler(r, p, y):
    # … your conversion …

def send_attitude_target(r, p, y, thrust=0.5):
    # … your MAVLink …

def compute_bearing(lat1, lon1, lat2, lon2):
    # … your bearing …

async def control_loop():
    commanded_yaw = 0.0
    last_update   = time.time()
    while True:
        send_attitude_target(0, -math.radians(5), commanded_yaw)
        if time.time() - last_update > 3.0 and current_pos and target_pos:
            last_update = time.time()
            try:
                lat1, lon1 = current_pos['lat'], current_pos['lon']
                lat2, lon2 = target_pos ['lat'], target_pos ['lon']
                dist = math.hypot(lat2-lat1, lon2-lon1)*111000
                yaw  = compute_bearing(lat1, lon1, lat2, lon2)
                print(f"→ Δ={dist:.0f}m, yaw={math.degrees(yaw):.1f}°")
                commanded_yaw = yaw
            except Exception:
                print("‼️ Exception in guidance update:")
                traceback.print_exc()
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
