# Raspberry Pi head-less ArduPilot SITL + MAVLink-router + gps.py helper  
_works with Pi OS Bookworm 64-bit, user **cdc**_

---

## 0  Prepare the Pi (~10 min)

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git curl wget build-essential ccache g++ make                     python3 python3-venv python3-pip python3-pexpect                     pkg-config libtool autoconf meson ninja-build                     libxml2-dev libxslt1-dev zlib1g-dev                     libglib2.0-dev libdbus-1-dev libsystemd-dev
sudo reboot
```

---

## 1  Clone the project (gps helper + test page)

```bash
cd $HOME
git clone https://github.com/dezertefer/CustomGPSdata.git
# gps.py is now at  ~/CustomGPSdata/gps.py
```

---

## 2  Python virtual-env for **gps.py** and tooling

```bash
python3 -m venv ~/venv-ardupilot
source ~/venv-ardupilot/bin/activate

pip install --upgrade pip
pip install pymavlink websockets empy==3.3.4 future python-dateutil

deactivate          # re‑enter only when running gps.py
```

---

## 3  Build ArduPilot SITL (one‑time, ~25 min on Pi 4)

```bash
cd $HOME
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

./Tools/environment_install/install-prereqs-ubuntu.sh -y
exec $SHELL                        # reload env vars

pip install pexpect
pip install empy==3.3.4 --break-system-packages

./waf configure --board sitl
./waf copter -j$(nproc)            # first build ≈ 25 min
```

Binary produced:  
`~/ardupilot/build/sitl/bin/arducopter`

---

## 4  Systemd service — **arducopter-sitl**

```bash
sudo tee /etc/systemd/system/arducopter-sitl.service >/dev/null <<'EOF'
[Unit]
Description=ArduCopter SITL (headless, no MAVProxy)
After=network.target

[Service]
User=cdc
Group=cdc
WorkingDirectory=/home/cdc/ardupilot

ExecStart=/home/cdc/venv-ardupilot/bin/python3           /home/cdc/ardupilot/Tools/autotest/sim_vehicle.py           -v ArduCopter -f quad           --speedup 1           -N --out=udp:127.0.0.1:14550           --no-mavproxy  -l 39.417790,-76.615905,35,0 

Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF
```

---

## 5  Build & install **mavlink-router**

```bash
cd $HOME
git clone https://github.com/mavlink-router/mavlink-router.git
cd mavlink-router
git submodule update --init --recursive

meson setup build .
sudo ninja -C build install          # installs /usr/bin/mavlink-routerd
```

### 5‑A  `/etc/mavlink-router/main.conf`

```bash
sudo mkdir -p /etc/mavlink-router
sudo tee /etc/mavlink-router/main.conf >/dev/null <<'EOF'
[General]
TcpServerPort=0
ReportStats=false
MavlinkDialect=common

[UdpEndpoint SITL]
Mode    = Server
Address = 127.0.0.1
Port    = 14550

[UdpEndpoint GPS_PY]
Mode    = Normal
Address = 127.0.0.1
Port    = 15550

[UdpEndpoint GroundStation]
Mode    = Normal
Address = 192.168.195.175
Port    = 16550

[TcpEndpoint SITL_TCP]
Mode      = Normal
Address   = 127.0.0.1
Port      = 5760
Reconnect = True
EOF
```

---

## 6  Systemd service — **gps-drone** (`gps.py`)

```bash
sudo tee /etc/systemd/system/gps-drone.service >/dev/null <<'EOF'
[Unit]
Description=GUIDED_NOGPS attitude helper
After=mavlink-router.service
Wants=mavlink-router.service

[Service]
Type=simple
User=cdc
Group=cdc
WorkingDirectory=/home/cdc/CustomGPSdata
Environment=PYTHONUNBUFFERED=1
ExecStart=/home/cdc/venv-ardupilot/bin/python3 gps.py
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
```

---

## 7  Enable & start everything

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now arducopter-sitl.service
sudo systemctl enable --now mavlink-router.service
sudo systemctl enable --now gps-drone.service
```

Watch logs:

```bash
journalctl -u arducopter-sitl.service -f
journalctl -u mavlink-router.service -f
journalctl -u gps-drone.service   -f
```

---

## 8  Quick test in a browser

1. Open `~/CustomGPSdata/ws_client.html`  
   (or copy to your PC and set **Server IP** to the Pi’s address).
2. Press **Connect** – both sockets show ✅.
3. In Mission Planner change to **GUIDED_NOGPS**.  
   `gps.py` starts streaming `SET_ATTITUDE_TARGET`.

---

Enjoy your virtual flights! ✈️
