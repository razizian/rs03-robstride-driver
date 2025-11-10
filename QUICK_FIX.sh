#!/bin/bash
# Quick diagnostic and fix script

echo "RS03 Quick Diagnostic"
echo "===================="

echo -e "\n1. Checking USB devices..."
if lsusb | grep -q "0483:df11"; then
    echo "⚠️  MKS CANable is in DFU mode!"
    echo "   Try unplugging and replugging WITHOUT holding any buttons"
elif lsusb | grep -q "1a86"; then
    echo "✓ Found CH340 USB serial device"
else
    echo "✗ No CAN adapter detected"
fi

echo -e "\n2. Looking for serial devices..."
for dev in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$dev" ]; then
        echo "✓ Found: $dev"
        export FOUND_DEV=$dev
    fi
done

if [ -z "$FOUND_DEV" ]; then
    echo "✗ No serial devices found"
    echo ""
    echo "TROUBLESHOOTING:"
    echo "1. Unplug MKS CANable"
    echo "2. Wait 5 seconds"  
    echo "3. Plug back in (don't press any buttons)"
    echo "4. Run this script again"
    exit 1
fi

echo -e "\n3. Checking CAN interface..."
if ip link show can0 2>/dev/null | grep -q "can0"; then
    echo "✓ can0 exists"
else
    echo "✗ can0 not found"
    echo ""
    echo "Setting up CAN with device: $FOUND_DEV"
    echo "Run: sudo ./scripts/can_up.sh"
    echo "Or manually:"
    echo "  sudo slcand -o -c -s8 $FOUND_DEV can0"
    echo "  sudo ip link set can0 up"
fi

echo -e "\n4. Quick CAN test..."
echo "If CAN is up, run:"
echo "  uv run python debug_hardware.py"
echo ""
echo "This will scan for your motor's ID automatically!"

