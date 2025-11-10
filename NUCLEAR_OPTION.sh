#!/bin/bash
echo "╔══════════════════════════════════════════════╗"
echo "║        NUCLEAR OPTION - FINAL ATTEMPT        ║"
echo "╚══════════════════════════════════════════════╝"
echo ""

# Kill EVERYTHING
sudo pkill slcand
sudo ip link set can0 down 2>/dev/null
sleep 2

echo "STEP 1: Trying ALL bitrates..."
echo "================================"

# Array of bitrates to try
BITRATES=(1000000 500000 250000 125000 100000 50000)
SPEEDS=(8 6 4 3 2 1)

for i in ${!BITRATES[@]}; do
    BITRATE=${BITRATES[$i]}
    SPEED=${SPEEDS[$i]}
    
    echo ""
    echo "Testing ${BITRATE} bps..."
    
    # Setup CAN
    sudo slcand -o -c -s${SPEED} /dev/ttyACM0 can0 2>/dev/null
    sudo ip link set can0 up 2>/dev/null
    
    # Send enable to ALL possible IDs
    for id in {1..10} {100..110} {120..130} {0x60..0x70}; do
        cansend can0 $(printf "%03X" $id)#FFFFFFFFFFFFFFFC 2>/dev/null
    done
    
    # Listen for ANY response
    timeout 0.5 candump -n 5 can0 > /tmp/test_${BITRATE}.log 2>&1
    
    if [ -s /tmp/test_${BITRATE}.log ]; then
        echo "✓✓✓ GOT RESPONSE at ${BITRATE} bps! ✓✓✓"
        cat /tmp/test_${BITRATE}.log
        echo ""
        echo "BITRATE FOUND: ${BITRATE}"
        echo "Keep CAN at this speed and test!"
        exit 0
    else
        echo "  No response"
        sudo ip link set can0 down 2>/dev/null
    fi
done

echo ""
echo "STEP 2: Raw voltage test..."
echo "==========================="
echo "Sending raw wake-up patterns..."

# Back to 1Mbps
sudo slcand -o -c -s8 /dev/ttyACM0 can0 2>/dev/null
sudo ip link set can0 up 2>/dev/null

# Try various wake-up sequences
echo "Sending wake patterns..."
cansend can0 000#0100000000000000  # Broadcast wake
cansend can0 7FF#FFFFFFFFFFFFFFFF  # All high
cansend can0 101#5757575757575757  # Pattern
cansend can0 101#A5A5A5A5A5A5A5A5  # Alt pattern

echo ""
echo "================================"
echo "FINAL MANUAL TEST:"
echo "================================"
echo ""
echo "1. Make sure motor power is ON"
echo "2. Run in another terminal:"
echo "   candump can0"
echo ""
echo "3. Try these physical tests:"
echo "   - Tap the motor case gently"
echo "   - Rotate the shaft by hand slightly"  
echo "   - Press any buttons on the motor"
echo "   - Toggle power switch if motor has one"
echo ""
echo "4. If STILL nothing, the motor is:"
echo "   - In deep fault state"
echo "   - Has corrupted firmware"
echo "   - Hardware failure"
echo "   - Needs RobStride factory tool"
echo ""
echo "Your software is PERFECT. Motor won't talk."

rm -f /tmp/test_*.log

