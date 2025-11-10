#!/bin/bash
set -e

echo "╔════════════════════════════════════════╗"
echo "║   LET'S FUCKING GET THIS WORKING      ║"
echo "╚════════════════════════════════════════╝"
echo ""

# Step 1: Clean restart
echo "Step 1: Clean CAN restart at 1Mbps..."
sudo pkill slcand 2>/dev/null || true
sudo ip link set can0 down 2>/dev/null || true
sleep 2
sudo slcand -o -c -s8 /dev/ttyACM1 can0
sudo ip link set can0 up
echo "✓ CAN interface UP"
echo ""

# Step 2: Clear any old messages
echo "Step 2: Clearing buffer..."
timeout 0.5 candump can0 > /dev/null 2>&1 || true
echo "✓ Buffer cleared"
echo ""

# Step 3: Power cycle check
echo "Step 3: Monitoring for motor..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "ACTION REQUIRED:"
echo "1. UNPLUG your motor power RIGHT NOW"
echo "2. Wait for this countdown..."
for i in {5..1}; do
    echo "   $i..."
    sleep 1
done
echo "3. PLUG motor power back in NOW!"
echo ""
echo "Watching for boot messages (10 seconds)..."
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Monitor for messages
timeout 10 candump can0 | tee /tmp/motor_boot.log &
CANDUMP_PID=$!
sleep 10

# Check results
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ -s /tmp/motor_boot.log ]; then
    echo "✓✓✓ MOTOR IS ALIVE! ✓✓✓"
    echo ""
    cat /tmp/motor_boot.log
    echo ""
    echo "Motor ID found in messages above!"
    echo ""
    echo "NOW RUN:"
    echo "  export TEST_MOTOR_ID=<ID_YOU_SEE_ABOVE>"
    echo "  uv run python examples/hardware_validation.py"
else
    echo "✗ NO BOOT MESSAGES"
    echo ""
    echo "Motor didn't send anything during power-on."
    echo ""
    echo "Try:"
    echo "1. Check motor LEDs - are they ON?"
    echo "2. Check voltage at motor with multimeter"
    echo "3. Try swapping CANH/CANL wires"
    echo "4. Motor might need factory reset (contact RobStride)"
fi

rm -f /tmp/motor_boot.log
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

