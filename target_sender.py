#!/usr/bin/env python3
"""
target_sender.py
----------------
• Connects to ws://localhost:8766
• Sends the next waypoint in WAYPOINTS[]
  only after the server replies {"reached": true}
"""

import asyncio, websockets, json, time

WAYPOINTS = [
    (-35.363262, 149.165237),      # home
    (-35.3187709, 149.1570670),
    (-35.3110088, 149.2113521),
    (-35.3552028, 149.2211129),
]

async def waypoint_driver():
    uri = "ws://localhost:8766"
    print("connecting target_sender →", uri)
    async with websockets.connect(uri) as ws:
        print("connected")

        index = 0
        # send the very first corner immediately (index 1 = second point)
        await ws.send(json.dumps({"lat": WAYPOINTS[1][0], "lon": WAYPOINTS[1][1]}))
        print(f"[{time.strftime('%X')}] sent → wp1")

        while True:
            msg = await ws.recv()
            try:
                data = json.loads(msg)
            except json.JSONDecodeError:
                print("⛔ non-JSON reply:", msg)
                continue

            if data.get("reached"):
                # advance to next corner
                index = (index + 1) % len(WAYPOINTS)
                tgt_lat, tgt_lon = WAYPOINTS[(index + 1) % len(WAYPOINTS)]
                await ws.send(json.dumps({"lat": tgt_lat, "lon": tgt_lon}))
                print(f"[{time.strftime('%X')}] sent → wp{(index+1)%len(WAYPOINTS)}")

if __name__ == "__main__":
    asyncio.run(waypoint_driver())
