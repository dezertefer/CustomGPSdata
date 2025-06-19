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

3. Create an Ardupilot sitl service

# 3-A. Add service file
sudo tee /etc/systemd/system/arducopter-sitl.service > /dev/null <<'EOF'                               
[Unit]
Description=ArduCopter SITL (headless, no MAVProxy)
After=network.target

[Service]
# run as your normal user so log files, parameters, etc. land in /home/<user>
User=home
Group=home
WorkingDirectory=/home/home/ardupilot
# path to your ArduPilot repo

ExecStart=/home/home/venv-ardupilot/bin/python3  \
          /home/home/ardupilot/Tools/autotest/sim_vehicle.py \
          -v ArduCopter               \
          -f quad                     \
          --speedup 1                 \
          -N --out=udp:127.0.0.1:14550   \
          --no-mavproxy

Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target

EOF

Tip:
the first full build is long; afterwards you may add --skip-build
to sim_vehicle.py so the service launches instantly.

# activate the ArduPilot venv
source /home/home/venv-ardupilot/bin/activate

# install pexpect (and, automatically, ptyprocess)
pip install --no-cache-dir pexpect
# you can leave the venv with:  deactivate

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
sudo ninja -C build install

# 4) create main.conf for mavlink-router app

sudo mkdir -p /etc/mavlink-router

sudo tee /etc/mavlink-router/main.conf >/dev/null <<'EOF'
[General]
# Disable the built-in TCP server (so SITL can bind 5760 itself)
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

Changing this command you can change IP of GCS or add more endpoints if needed!


