#!/bin/bash
echo "BASIC HARDWARE CHECK"
echo "===================="
echo ""

echo "1. CAN Interface Status:"
ip link show can0 | grep -E "(state|bitrate)"
echo ""

echo "2. CAN Statistics:"
ip -s link show can0 | grep -A1 "TX:"
echo ""

echo "3. USB Device:"
ls -l /dev/ttyACM* 2>/dev/null || echo "No ttyACM devices found"
echo ""

echo "4. Sending test message..."
cansend can0 101#0123456789ABCDEF
sleep 0.5

echo "5. Checking if we see our own message..."
timeout 2 candump can0 > /tmp/dump.txt 2>&1 &
sleep 0.5
cansend can0 101#FEEDBEEFDEADCAFE
sleep 1.5

if grep -q "101" /tmp/dump.txt; then
    echo "✓ CAN loopback working - we see our own messages"
else
    echo "✗ Not even seeing our own messages - CAN might be broken"
fi

echo ""
echo "NOW DO THIS:"
echo "1. With candump running: candump can0"
echo "2. UNPLUG motor power cable from motor"
echo "3. Wait 5 seconds"
echo "4. PLUG power cable back into motor"
echo ""
echo "You MUST see messages when motor boots, or:"
echo "  - Motor has NO power (check voltage with multimeter)"
echo "  - CAN wires not connected to motor"
echo "  - Motor is DEAD"

rm -f /tmp/dump.txt

