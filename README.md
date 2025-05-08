First we install sitl
sudo apt update && sudo apt upgrade -y
sudo apt install -y git python3 python3-pip python3-dev \
  build-essential ccache gawk wget unzip zip \
  libtool autoconf automake screen pkg-config libxml2-utils

pip3 install --user future pymavlink --break-system-packages

cd ~
git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules
cd ardupilot

Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.profile

./waf configure --board sitl
./waf copter

cd ArduCopter
sim_vehicle.py
