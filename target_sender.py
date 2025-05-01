#!/usr/bin/env python3
import asyncio, websockets, json, time

async def send_target(lat, lon, alt=584.0, interval=5.0):
    ws_url = "ws://localhost:8766"
    print(f"Connecting target sender → {ws_url}")
    async with websockets.connect(ws_url) as ws:
        print("Connected.")
        while True:
            payload = {'lat': lat, 'lon': lon, 'alt': alt}
            await ws.send(json.dumps(payload))
            print(f"[{time.strftime('%X')}] Sent target → {payload}")
            await asyncio.sleep(interval)

if __name__ == '__main__':
    # example fixed corner
    asyncio.run(send_target(-35.3187709, 149.1570670))
