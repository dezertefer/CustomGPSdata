0. Prepare the Pi (≈10 min)

sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git curl screen tmux \
                    python3 python3-venv python3-pip \
                    build-essential ccache g++ wget \
                    pkg-config libtool autoconf \
                    libxml2-dev libxslt1-dev zlib1g-dev
sudo reboot

1. Clone your project

cd $HOME
git clone https://github.com/dezertefer/CustomGPSdata.git

We will keep gps.py and the web/ test page exactly where the repo puts them:
~/CustomGPSdata/…

2. Build ArduPilot SITL (one-time, ≈25 min on Pi 4)

# 2-A.  Get the source
cd $HOME
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# 2-B.  Pull the official prerequisites script:
./Tools/environment_install/install-prereqs-ubuntu.sh -y
# shell needs to pick up new env vars
exec $SHELL

# 2-C. Install the most critical dependencies separately
sudo python3 -m pip install --break-system-packages empy==3.3.4 future python-dateutil

# 2-D.  One-off build (quad-copter firmware)
./waf configure --board sitl
./waf copter  -j$(nproc)

Tip:
the first full build is long; afterwards you may add --skip-build
to sim_vehicle.py so the service launches instantly.

3. Python virtual-env for gps.py
python3 -m venv ~/venv-ardupilot
source ~/venv-ardupilot/bin/activate
pip install --upgrade pip
pip install pymavlink websockets
deactivate

4. Install Mavlink-Router

# 1) prerequisites ────────────────────────────────────────────────
sudo apt update
sudo apt install -y \
      git meson ninja-build pkg-config gcc g++ \
      systemd libsystemd-dev libglib2.0-dev libdbus-1-dev zlib1g-dev

# 2) get the source ───────────────────────────────────────────────
cd ~
git clone https://github.com/mavlink-router/mavlink-router.git
cd mavlink-router
git submodule update --init --recursive

# 3) configure & compile ──────────────────────────────────────────
meson setup build .
ninja -C build          # -j$(nproc) is implied

# 4) install system-wide (to /usr/local) ──────────────────────────
sudo ninja -C build install
sudo ldconfig           # refresh shared-library cache

# 5) create a systemd service (optional) ──────────────────────────
sudo tee /etc/systemd/system/mavlink-router.service >/dev/null <<'EOF'
[Unit]
Description=MAVLink Router
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/mavlink-routerd --syslog
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now mavlink-router
