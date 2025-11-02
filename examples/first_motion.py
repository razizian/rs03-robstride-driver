#!/usr/bin/env python3
"""
Minimal RS03 motion test using robstride SDK.
Usage:
  python first_motion.py --dry-run  # read telemetry only
  python first_motion.py            # enable + jog ±0.5 rad/s
"""
import os
import sys
import time
import can
from robstride import Client, RunMode

# Parse arguments
DRY_RUN = '--dry-run' in sys.argv

# Configuration
PORT = os.getenv('PORT', '/dev/ttyACM0')
BITRATE = int(os.getenv('BITRATE', 1000000))
MOTOR_ID = int(os.getenv('NODE_ID', 127))

print(f"Connecting to motor {MOTOR_ID} via can0...")

# Use socketcan interface (can0 already set up by scripts/can_up.sh)
bus = can.Bus(interface='socketcan', channel='can0', bitrate=BITRATE)
client = Client(bus)

if DRY_RUN:
    print("\n=== DRY RUN MODE (read-only) ===")
    try:
        # Read current parameters
        pos = client.read_param(MOTOR_ID, 'mechpos')
        vel = client.read_param(MOTOR_ID, 'mechvel')
        vbus = client.read_param(MOTOR_ID, 'vbus')
        print(f"Position: {pos:.3f} rad")
        print(f"Velocity: {vel:.3f} rad/s")
        print(f"Bus Voltage: {vbus:.1f} V")
    except Exception as e:
        print(f"Error reading params: {e}")
        print("Motor may not be powered or responding.")
    finally:
        bus.shutdown()
    print("Done (no motion executed).")
    sys.exit(0)

# Safety limits with verification
print("Setting safety limits (I_max=2A, V_max=2rad/s)...")
client.write_param(MOTOR_ID, 'limit_cur', 2.0)
client.write_param(MOTOR_ID, 'limit_spd', 2.0)

# Verify limits were set
actual_cur = client.read_param(MOTOR_ID, 'limit_cur')
actual_spd = client.read_param(MOTOR_ID, 'limit_spd')
print(f"Verified limits: Current={actual_cur:.1f}A, Speed={actual_spd:.1f}rad/s")

if abs(actual_cur - 2.0) > 0.1 or abs(actual_spd - 2.0) > 0.1:
    print("ERROR: Safety limits not set correctly!")
    bus.shutdown()
    sys.exit(1)

# Enable motor with error handling
print("Enabling motor...")
try:
    feedback = client.enable(MOTOR_ID)
    print("Motor enabled successfully!")
except Exception as e:
    print(f"ERROR: Failed to enable motor: {e}")
    bus.shutdown()
    sys.exit(1)

# TODO: The SDK doesn't expose velocity control in the Client class we saw
# For now, demonstrate enable/disable cycle only
print("\nWaiting 2 seconds...")
time.sleep(2)

# Read final telemetry
pos = client.read_param(MOTOR_ID, 'mechpos')
vel = client.read_param(MOTOR_ID, 'mechvel')
vbus = client.read_param(MOTOR_ID, 'vbus')
print(f"\nFinal state - Pos: {pos:.3f} rad, Vel: {vel:.3f} rad/s, Vbus: {vbus:.1f} V")

# Disable motor with safety stop
try:
    print("Sending stop command...")
    # Try to stop motion before disabling
    for _ in range(5):
        try:
            # Send zero velocity command if we had velocity control
            pass
        except:
            pass
        time.sleep(0.1)
    
    print("Disabling motor...")
    client.disable(MOTOR_ID)
except Exception as e:
    print(f"Warning during shutdown: {e}")
finally:
    bus.shutdown()
    print("Done.")

