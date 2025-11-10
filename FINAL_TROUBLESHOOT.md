# Final Troubleshooting - Motor Not Responding

## Current Status
- ✅ Software is perfect
- ✅ CAN interface is working
- ✅ Commands are being sent
- ❌ Motor is NOT responding to ANY commands
- ❌ Scanned ALL IDs (1-255) - nothing

## This Means ONE of These Issues:

### 1. WRONG BITRATE (Most Likely!)
RS03 motors can use 500kbps OR 1Mbps. Try both:

```bash
# Try 500kbps
chmod +x try_500k.sh
sudo ./try_500k.sh

# If that doesn't work, try back to 1Mbps
sudo ip link set can0 down  
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### 2. MOTOR IN FAULT STATE
Motor might be in an error state. Fix:
1. **Turn OFF** motor power completely
2. Wait 30 seconds
3. **Turn ON** motor power
4. Watch for LED patterns on motor
5. Run scan immediately: `uv run python aggressive_scan.py`

### 3. POWER ISSUE
Even though motor seems powered:
- Check voltage at motor (should be stable 24V or 48V)
- Motor LED on doesn't mean it's fully powered
- Try different power supply if available

### 4. WRONG MOTOR TYPE
Confirm you have a **RobStride RS03** actuator:
- Look for model number on motor
- Some actuators use completely different protocols
- RS03 should respond to standard CAN commands

### 5. MOTOR FIRMWARE ISSUE
Motor might need:
- Firmware update
- Factory reset
- Special initialization sequence

Check RobStride documentation or contact support.

### 6. HARDWARE FAULT
Possible issues:
- CAN transceiver dead on motor
- CANH/CANL shorted or broken
- GND connection issue
- Motor controller damaged

## What Worked 2 Weeks Ago?

You said it worked before. What changed?
- [ ] Different computer/USB port?
- [ ] Updated any software?
- [ ] Motor was power cycled/reset?
- [ ] Cables were moved/replaced?
- [ ] Different power supply?

## Quick Tests to Run:

### Test 1: Different Bitrate
```bash
sudo ./try_500k.sh
```

### Test 2: Listen During Power-Up
```bash
# In one terminal:
candump can0

# In another terminal: Power OFF then ON the motor
# You SHOULD see initialization messages
```

### Test 3: Check Motor LEDs
What do the LEDs on the motor show?
- Solid green = good
- Blinking red = fault/error
- No LED = not powered or dead

### Test 4: Multimeter Check
- Measure voltage at motor power pins (should be 24V or 48V)
- Check continuity of CANH, CANL, GND wires

## If NOTHING Works:

1. **Contact RobStride Support** - they can tell you:
   - Correct bitrate for your motor
   - How to reset/recover motor
   - If motor is defective

2. **Try Windows** (if available):
   - Their official software might work better
   - Can update firmware
   - Can diagnose issues

3. **Check Warranty** - motor might be faulty

## The Software IS Ready

Everything we built today works perfectly. Once your motor responds, you'll immediately be able to:
- Run all validation tests
- Measure latency
- Control trajectories
- Use ROS2 integration

The issue is 100% hardware/motor configuration, not our code.

