#!/usr/bin/env python3
"""
current_position_simulator_square.py
-----------------------------------
Connects to ws://localhost:8765 and emits one JSON fix/sec that:
  • starts at the “home” vertex,
  • travels side-to-side around the 5 km × 5 km box,
  • moves 2.6 m/s along the current leg,
  • adds ±250 m noise each fix to simulate 500 m GPS resolution.
"""

import asyncio, websockets, json, math, random, time

# ---- 5 km × 5 km flight-plan vertices (same order as earlier) ----
WAYPOINTS = [
    (-35.3632620, 149.1652370),  # home
    (-35.3187709, 149.1570670),
    (-35.3110088, 149.2113521),
    (-35.3552028, 149.2211129),
]

SPEED = 10            # m/s ground-speed
SEND_INTERVAL = 1.0    # seconds between fixes
NOISE_RADIUS = 250     # metres (half of 500 m “grid”)

def bearing_rad(lat1, lon1, lat2, lon2):
    """bearing in radians from (lat1,lon1) to (lat2,lon2)"""
    dLon = math.radians(lon2 - lon1)
    φ1, φ2 = map(math.radians, (lat1, lat2))
    x = math.sin(dLon)*math.cos(φ2)
    y = math.cos(φ1)*math.sin(φ2) - math.sin(φ1)*math.cos(φ2)*math.cos(dLon)
    brng = math.atan2(x, y)
    return brng if brng >= 0 else brng + 2*math.pi

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6_371_000
    φ1, φ2 = map(math.radians, (lat1, lat2))
    dφ  = math.radians(lat2 - lat1)
    dλ  = math.radians(lon2 - lon1)
    a = math.sin(dφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(dλ/2)**2
    return 2*R*math.asin(math.sqrt(a))

def step_latlon(lat, lon, bearing, distance_m):
    """move `distance_m` along `bearing` (rad) on the sphere"""
    R = 6_371_000
    δ = distance_m / R
    φ1, λ1 = math.radians(lat), math.radians(lon)
    φ2 = math.asin(math.sin(φ1)*math.cos(δ) + math.cos(φ1)*math.sin(δ)*math.cos(bearing))
    λ2 = λ1 + math.atan2(math.sin(bearing)*math.sin(δ)*math.cos(φ1),
                         math.cos(δ)-math.sin(φ1)*math.sin(φ2))
    return math.degrees(φ2), math.degrees(λ2)

async def simulate():
    ws_url = "ws://localhost:8765"
    print("Connecting simulator →", ws_url)
    async with websockets.connect(ws_url) as ws:
        print("Connected.")

        wp_index = 0
        lat, lon = WAYPOINTS[0]
        while True:
            # distance to current target
            tgt_lat, tgt_lon = WAYPOINTS[(wp_index + 1) % len(WAYPOINTS)]
            dist_to_wp = haversine_m(lat, lon, tgt_lat, tgt_lon)

            if dist_to_wp < SPEED*SEND_INTERVAL:        # reached vertex → next leg
                wp_index = (wp_index + 1) % len(WAYPOINTS)
                tgt_lat, tgt_lon = WAYPOINTS[(wp_index + 1) % len(WAYPOINTS)]
                dist_to_wp = haversine_m(lat, lon, tgt_lat, tgt_lon)

            bearing = bearing_rad(lat, lon, tgt_lat, tgt_lon)
            lat, lon = step_latlon(lat, lon, bearing, SPEED*SEND_INTERVAL)

            # add ±250 m random “jump”
            lat_err = random.uniform(-NOISE_RADIUS, NOISE_RADIUS) / 111_000
            cos_lat = math.cos(math.radians(lat))
            lon_err = random.uniform(-NOISE_RADIUS, NOISE_RADIUS) / (111_000 * max(cos_lat, 1e-6))
            noisy_lat, noisy_lon = lat + lat_err, lon + lon_err

            payload = {"lat": noisy_lat, "lon": noisy_lon}
            await ws.send(json.dumps(payload))
            print(f"[{time.strftime('%X')}] sent → {payload}")
            await asyncio.sleep(SEND_INTERVAL)

if __name__ == "__main__":
    asyncio.run(simulate())
