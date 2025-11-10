# Hardware Debugging - Software is Working! ✅

Your software setup is **PERFECT**. The CAN interface sent 11 packets trying to find your motor. The issue is with the physical connections.

## Quick Status
- ✅ MKS CANable working (green/blue lights)
- ✅ CAN interface up and transmitting  
- ✅ Software scanning for motors correctly
- ❌ No response from RS03 motor

## Check These NOW:

### 1. Motor Power 🔌
- Is the motor power supply ON? 
- Check voltage with multimeter (should be 12-48V DC)
- Is the power LED on the motor lit?

### 2. Wiring Connections 🔗
Double-check EVERY connection:
```
MKS CANable    →    RS03 Motor
-----------         -----------
CANH          →     CANH (or CAN_H)
CANL          →     CANL (or CAN_L)  
GND           →     GND
```

### 3. Termination Resistor ⚡
- Do you have a 120Ω resistor across CANH and CANL?
- Try adding one at the motor end
- Some motors have built-in termination (check docs)

### 4. Common Issues:

**Swapped Wires:** Try swapping CANH/CANL:
```
CANH → CANL
CANL → CANH
```

**Bad Connection:** 
- Wiggle all connectors
- Check for loose wires
- Ensure good contact

**Wrong Bitrate:** Your motor might use different speed:
```bash
# Try 500kbps instead of 1Mbps
sudo ip link set can0 down
sudo slcand -o -c -s6 /dev/ttyACM1 can0  # s6 = 500kbps
sudo ip link set can0 up
uv run python debug_hardware.py
```

### 5. Motor Test
With motor powered, you should:
- See LEDs on the motor
- Hear/feel it initialize on power-up
- Sometimes motors beep or twitch

### 6. Live Monitoring
Keep this running in another terminal:
```bash
candump can0
```
Then power cycle the motor - you should see messages!

## If Still No Response:
1. The motor ID might not be 101-127
2. Motor might be in a fault state (power cycle it)
3. Try a different CAN cable
4. Motor firmware might need different commands

**Your software is ready - it's just waiting for the motor to talk back!**

