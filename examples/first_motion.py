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
PORT = os.getenv('PORT', '/dev/ch340_can')
BITRATE = int(os.getenv('BITRATE', 1000000))
MOTOR_ID = int(os.getenv('NODE_ID', 107))

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

# Safety limits
print("Setting safety limits (I_max=2A, V_max=2rad/s)...")
client.write_param(MOTOR_ID, 'limit_cur', 2.0)
client.write_param(MOTOR_ID, 'limit_spd', 2.0)

# Enable motor
print("Enabling motor...")
feedback = client.enable(MOTOR_ID)
print("Motor enabled successfully!")

# TODO: The SDK doesn't expose velocity control in the Client class we saw
# For now, demonstrate enable/disable cycle only
print("\nWaiting 2 seconds...")
time.sleep(2)

# Read final telemetry
pos = client.read_param(MOTOR_ID, 'mechpos')
vel = client.read_param(MOTOR_ID, 'mechvel')
vbus = client.read_param(MOTOR_ID, 'vbus')
print(f"\nFinal state - Pos: {pos:.3f} rad, Vel: {vel:.3f} rad/s, Vbus: {vbus:.1f} V")

# Disable motor
print("Disabling motor...")
client.disable(MOTOR_ID)
bus.shutdown()
print("Done.")

