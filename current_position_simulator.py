#!/usr/bin/env python3
import asyncio, websockets, json, time, math, random

async def simulate_current(lat, lon, alt=584.0, interval=1.0):
    """
    Simulate a drone moving north at 5 m/s, but report position with ±250 m random noise.
    """
    ws_url = "ws://localhost:8765"
    speed = 5.0  # m/s
    print(f"Connecting simulator → {ws_url}")
    async with websockets.connect(ws_url) as ws:
        print("Connected.")
        while True:
            # move northward
            delta_lat = (speed * interval) / 111000.0
            lat += delta_lat

            # add ±250 m noise (~step/2)
            lat_step = 500/111000.0
            cos_lat  = math.cos(math.radians(lat))
            lon_step = 500/(111000.0 * cos_lat) if cos_lat else 0.005

            noisy_lat = lat + random.uniform(-lat_step/2, lat_step/2)
            noisy_lon = lon + random.uniform(-lon_step/2, lon_step/2)

            payload = {'lat': noisy_lat, 'lon': noisy_lon, 'alt': alt}
            await ws.send(json.dumps(payload))
            print(f"[{time.strftime('%X')}] Sent current → {payload}")
            await asyncio.sleep(interval)

if __name__ == '__main__':
    # starting point = home
    asyncio.run(simulate_current(-35.363262, 149.165237))
