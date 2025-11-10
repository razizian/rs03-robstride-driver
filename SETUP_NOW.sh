#!/bin/bash
# Quick setup for ttyACM1

echo "MKS CANable found at /dev/ttyACM1!"
echo "==================================="
echo ""
echo "Run these commands to set up CAN:"
echo ""
echo "1. Set the device port:"
echo "   export PORT=/dev/ttyACM1"
echo ""
echo "2. Bring up CAN interface:"
echo "   sudo PORT=/dev/ttyACM1 ./scripts/can_up.sh"
echo ""
echo "Or manually:"
echo "   sudo slcand -o -c -s8 /dev/ttyACM1 can0"
echo "   sudo ip link set can0 up"
echo ""
echo "3. Then test your motor:"
echo "   uv run python debug_hardware.py"

