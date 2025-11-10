#!/bin/bash
echo "Clean Restart - 1Mbps"
echo "===================="

# Kill everything
echo "1. Stopping CAN interface..."
sudo pkill slcand 2>/dev/null
sudo ip link set can0 down 2>/dev/null

sleep 2

# Restart at 1Mbps
echo "2. Starting CAN at 1Mbps on /dev/ttyACM1..."
sudo slcand -o -c -s8 /dev/ttyACM1 can0
sleep 1
sudo ip link set can0 up

echo "3. CAN Status:"
ip link show can0

echo ""
echo "✓ Ready at 1Mbps"
echo ""
echo "NOW:"
echo "1. Power cycle your motor (OFF, wait 10 sec, ON)"
echo "2. Run: candump can0"
echo "3. You MUST see boot messages or motor isn't working"

