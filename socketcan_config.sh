sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.75 dbitrate 5000000 dsample-point 0.75 fd on
sudo ip link set can0 up