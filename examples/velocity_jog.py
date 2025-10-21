#!/usr/bin/env python3
"""
Velocity jog test for RS03 motor.
Jogs motor at ±0.5 rad/s using Speed mode.
"""
import os
import sys
import time
import can
from robstride import Client, RunMode

# Configuration
MOTOR_ID = int(os.getenv('NODE_ID', 127))
VELOCITY = 0.5  # rad/s

print(f"Connecting to motor {MOTOR_ID} via can0...")
bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
client = Client(bus)

try:
    # Read initial state
    print("\nInitial state:")
    pos = client.read_param(MOTOR_ID, 'mechpos')
    mode = client.read_param(MOTOR_ID, 'run_mode')
    print(f"  Position: {pos:.3f} rad")
    print(f"  Run mode: {mode}")
    
    # Set safety limits
    print("\nSetting safety limits...")
    client.write_param(MOTOR_ID, 'limit_cur', 2.0)
    client.write_param(MOTOR_ID, 'limit_spd', 2.0)
    
    # Enable motor
    print("Enabling motor...")
    client.enable(MOTOR_ID)
    
    # Switch to Speed mode
    print("Setting Speed mode...")
    client.write_param(MOTOR_ID, 'run_mode', RunMode.Speed)
    time.sleep(0.1)
    
    # Jog forward
    print(f"\nJogging at +{VELOCITY} rad/s for 2 seconds...")
    client.write_param(MOTOR_ID, 'spd_ref', VELOCITY)
    for i in range(4):
        time.sleep(0.5)
        vel = client.read_param(MOTOR_ID, 'mechvel')
        pos = client.read_param(MOTOR_ID, 'mechpos')
        print(f"  t={i*0.5:.1f}s: vel={vel:.3f} rad/s, pos={pos:.3f} rad")
    
    # Stop
    print("Stopping...")
    client.write_param(MOTOR_ID, 'spd_ref', 0.0)
    time.sleep(0.5)
    
    # Jog reverse
    print(f"Jogging at -{VELOCITY} rad/s for 2 seconds...")
    client.write_param(MOTOR_ID, 'spd_ref', -VELOCITY)
    time.sleep(2)
    
    # Stop
    print("Stopping...")
    client.write_param(MOTOR_ID, 'spd_ref', 0.0)
    time.sleep(0.5)
    
    # Read final position
    pos = client.read_param(MOTOR_ID, 'mechpos')
    print(f"\nFinal position: {pos:.3f} rad")
    
finally:
    # Always disable motor
    print("\nDisabling motor...")
    client.disable(MOTOR_ID)
    bus.shutdown()
    print("Done!")
