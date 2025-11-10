#!/bin/bash
echo "Trying 500kbps bitrate..."
echo "========================="

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

echo "✓ CAN set to 500kbps"
echo ""
echo "Now testing..."
uv run python aggressive_scan.py

