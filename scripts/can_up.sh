#!/bin/bash
sudo pkill slcand 2>/dev/null || true
sudo ip link delete can0 2>/dev/null || true
sudo systemctl stop ModemManager 2>/dev/null || true

# Attempt slcand path
sudo slcand -o -c -s8 /dev/ch340_can can0 && sudo ip link set can0 up
if ip link show can0 &>/dev/null; then
    echo "✓ can0 active at 1 Mbps"
    ip -details link show can0
    exit 0
fi

# Fallback: slcan_attach
echo "Trying slcan_attach..."
sudo slcan_attach -o -c -s8 /dev/ch340_can
IFACE=$(ip -br link | grep -E 'can|slcan' | grep -v DOWN | awk '{print $1}' | head -1)
if [ -n "$IFACE" ]; then
    sudo ip link set "$IFACE" up
    echo "✓ $IFACE active at 1 Mbps"
    ip -details link show "$IFACE"
else
    echo "✗ Failed to bring up CAN interface"
    exit 1
fi

