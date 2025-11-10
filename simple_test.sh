#!/bin/bash
# Super simple motor connection test

echo "=================================="
echo "RS03 SIMPLE CONNECTION TEST"
echo "=================================="
echo ""

# Check if motor has power
echo "❓ QUESTION 1: Is your motor power supply turned ON?"
echo "   Check if there are any LEDs lit on the motor itself."
echo ""
read -p "   Is motor powered? (y/n): " powered

if [[ ! $powered =~ ^[Yy]$ ]]; then
    echo ""
    echo "⚠️  MOTOR NEEDS POWER TO WORK!"
    echo "   Turn on your motor power supply (12-48V DC)"
    echo "   The motor should have status LEDs when powered"
    exit 1
fi

echo ""
echo "❓ QUESTION 2: Do you have these 3 wires connected?"
echo "   1. CANH (MKS CANable) → CANH (Motor)"
echo "   2. CANL (MKS CANable) → CANL (Motor)" 
echo "   3. GND (MKS CANable) → GND (Motor)"
echo ""
read -p "   All 3 wires connected? (y/n): " wired

if [[ ! $wired =~ ^[Yy]$ ]]; then
    echo ""
    echo "⚠️  YOU NEED ALL 3 WIRES CONNECTED!"
    echo "   CANH, CANL, and GND are required"
    exit 1
fi

echo ""
echo "❓ QUESTION 3: Do you have a 120Ω resistor?"
echo "   This goes across CANH and CANL at the motor end"
echo ""
read -p "   Have termination resistor? (y/n): " resistor

if [[ ! $resistor =~ ^[Yy]$ ]]; then
    echo ""
    echo "⚠️  YOU PROBABLY NEED THIS!"
    echo "   Without 120Ω termination, CAN bus may not work"
    echo "   Some motors have it built-in (check datasheet)"
fi

echo ""
echo "=================================="
echo "Running diagnostic..."
echo "=================================="
echo ""

# Test sending a message
echo "1. Testing CAN interface..."
if cansend can0 101#FF00FF00FF00FF00 2>/dev/null; then
    echo "   ✅ CAN interface can send messages"
else
    echo "   ❌ Cannot send CAN messages"
    exit 1
fi

echo ""
echo "2. Checking for any CAN traffic (10 seconds)..."
echo "   Power cycling your motor NOW might help..."
timeout 10 candump can0 > /tmp/can_test.log 2>&1 &
CANDUMP_PID=$!
sleep 10

if [ -s /tmp/can_test.log ]; then
    echo "   ✅ SAW CAN MESSAGES!"
    echo ""
    cat /tmp/can_test.log
    echo ""
    echo "=================================="
    echo "✅ HARDWARE IS CONNECTED!"
    echo "=================================="
    echo "Your motor is communicating!"
    echo ""
    echo "Now run: uv run python debug_hardware.py"
else
    echo "   ❌ NO CAN MESSAGES RECEIVED"
    echo ""
    echo "=================================="
    echo "⚠️  HARDWARE NOT RESPONDING"
    echo "=================================="
    echo ""
    echo "TROUBLESHOOTING:"
    echo "1. Double-check motor power is ON"
    echo "2. Verify all 3 wires (CANH, CANL, GND)"
    echo "3. Try swapping CANH and CANL wires"
    echo "4. Add 120Ω termination if missing"
    echo "5. Try different bitrate:"
    echo "   sudo ip link set can0 down"
    echo "   sudo ip link set can0 type can bitrate 500000"
    echo "   sudo ip link set can0 up"
fi

rm -f /tmp/can_test.log

