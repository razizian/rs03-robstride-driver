#!/bin/bash
sudo pkill slcand 2>/dev/null || true
sudo ip link delete can0 2>/dev/null || true
sudo systemctl stop ModemManager 2>/dev/null || true

# Check for MKS CANable device
DEVICE="/dev/ttyACM0"
if [ -e "$DEVICE" ]; then
    echo "Found MKS CANable at $DEVICE"
else
    echo "✗ No CAN adapter found at /dev/ttyACM0"
    exit 1
fi

# Attempt slcand
sudo slcand -o -c -s8 "$DEVICE" can0 && sudo ip link set can0 up
if ip link show can0 &>/dev/null; then
    echo "✓ can0 active at 1 Mbps"
    ip -details link show can0
    exit 0
fi

# Fallback: slcan_attach
echo "Trying slcan_attach..."
sudo slcan_attach -o -c -s8 "$DEVICE"
IFACE=$(ip -br link | grep -E 'can|slcan' | grep -v DOWN | awk '{print $1}' | head -1)
if [ -n "$IFACE" ]; then
    sudo ip link set "$IFACE" up
    echo "✓ $IFACE active at 1 Mbps"
    ip -details link show "$IFACE"
else
    echo "✗ Failed to bring up CAN interface"
    exit 1
fi

