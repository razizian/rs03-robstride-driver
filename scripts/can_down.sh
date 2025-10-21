#!/bin/bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set slcan0 down 2>/dev/null || true
sudo ip link delete can0 2>/dev/null || true
sudo pkill slcand 2>/dev/null || true
echo "CAN interfaces down"

