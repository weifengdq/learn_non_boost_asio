#!/bin/sh
# for WSL2
sudo modprobe can_raw
sudo modprobe vxcan

# 如果 ip link 中存在 vcan0, 就删除 vcan0
if ip link show vcan0 > /dev/null 2>&1; then
    sudo ip link set dev vcan0 down
    sudo ip link set dev vxcan0 down
    sudo ip link delete dev vcan0 type vcan
fi

sudo ip link add dev vcan0 type vxcan
sudo ip link set up vcan0
sudo ip link set dev vxcan0  up

# ip link