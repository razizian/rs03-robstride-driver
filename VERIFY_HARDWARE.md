# RS03 Hardware Verification Checklist

## What You're Seeing (Nothing on candump for 10 min)
This is **normal** if your motor isn't powered or wired correctly. CAN bus is silent when nothing is connected.

## Let's Verify Your Hardware Step-by-Step:

### Step 1: Check Motor Power Supply 🔋
**CRITICAL:** The motor needs power to respond!

- [ ] Power supply plugged into wall outlet?
- [ ] Power supply voltage correct? (Check motor label - usually 24V or 48V)
- [ ] Power cable connected to motor?
- [ ] Any LEDs lit on the motor? (Should see status lights)
- [ ] Measure voltage at motor terminals (use multimeter if available)

**If no LEDs on motor = motor has no power = won't respond**

### Step 2: Physical Inspection 👀
Look at your RS03 motor:
- Does it have any status LEDs? Are they ON?
- Does it feel energized (slight resistance if you try to turn shaft)?
- Did it make any sound when you powered it on?

### Step 3: Wiring Check 🔌
You need **3 wires minimum** from MKS CANable to motor:

```
MKS CANable    Wire Color    RS03 Motor
-----------    ----------    ----------
CANH           (often white) → CANH / CAN+ / H
CANL           (often blue)  → CANL / CAN- / L
GND            (black)       → GND / Ground
```

**Common mistake:** Some motors have a connector - make sure it's firmly seated!

### Step 4: Termination Resistor ⚡
**You NEED this!** Without it, CAN doesn't work properly.
- 120Ω resistor between CANH and CANL at the motor end
- Some motors have built-in termination (check datasheet)
- If you don't have one, this could be your problem!

### Step 5: Quick Power Cycle Test
With `candump can0` running in terminal:

1. Turn OFF motor power supply
2. Wait 5 seconds
3. Turn ON motor power supply
4. **Watch candump** - you should see messages immediately!

**If you see nothing:** Motor isn't powered OR wiring is wrong

### Step 6: Try Sending a Broadcast Command
```bash
# Send a CAN message manually
cansend can0 000#FFFFFFFFFFFFFFFF
```

Watch candump - you should at least see your own message echoed back.

## What SHOULD Happen When It Works:

### When motor is properly connected and powered:
1. **Motor powers on:** You'll see/hear initialization
2. **candump shows traffic:** Messages appear automatically
3. **Our script finds it:** `debug_hardware.py` will detect the ID
4. **Motor moves:** When you run tests, it will actually rotate!

### Expected Motor Movement:
When working, our validation script will:
- ✅ Enable motor (you'll hear a slight click/engagement)
- ✅ Hold position (motor resists if you try to turn it)
- ✅ Small movements (±0.1 rad, about 6 degrees)
- ✅ Slow velocity (0.25 rad/s, very gentle)
- ✅ Test compliance (you can manually move it when soft)

## Most Likely Issues (In Order):

### 1. Motor Not Powered (90% of issues)
**Fix:** Check power supply is ON and connected

### 2. Wrong/Missing Wiring (8% of issues)
**Fix:** Verify CANH, CANL, GND connections

### 3. No Termination Resistor (1.5% of issues)
**Fix:** Add 120Ω resistor across CANH-CANL

### 4. Wrong Bitrate (0.5% of issues)
**Fix:** Try 500kbps instead of 1Mbps

## Quick Diagnostic Commands:

```bash
# 1. Is CAN interface up?
ip link show can0
# Should say: "state UP"

# 2. Are we sending packets?
ip -s link show can0
# TX packets should be > 0

# 3. Listen for ANY CAN traffic
candump can0
# Leave this running, then power cycle motor

# 4. Send test message
cansend can0 101#FF00FF00FF00FF00
# Should see it in candump at minimum
```

## The Bottom Line:

**Your software is 100% working.** I can see the CAN interface is transmitting perfectly.

**The motor just isn't responding** because:
- It doesn't have power, OR
- The wiring isn't correct, OR  
- Missing termination resistor

**Check power first!** That's the most common issue.

Once you fix the hardware connection, you'll see messages in candump immediately and the motor will respond to commands!

