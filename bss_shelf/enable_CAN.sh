sudo modprobe can
sudo modprobe can-raw
sudo modprobe mttcan

sudo ip link set up can0 type can bitrate 1000000