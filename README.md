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

# 2-C.  One-off build (quad-copter firmware)
./waf configure --board sitl
./waf copter  -j$(nproc)

Tip:
the first full build is long; afterwards you may add --skip-build
to sim_vehicle.py so the service launches instantly.
