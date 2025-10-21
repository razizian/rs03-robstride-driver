#!/usr/bin/env python3
"""
Scan for RobStride motors on CAN bus by trying common IDs.
"""
import can
from robstride import Client

print("Scanning for motors on can0...")
bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
client = Client(bus, recv_timeout=0.5)  # Short timeout

# Try common IDs
for motor_id in [1, 2, 107, 127]:
    try:
        print(f"Trying ID {motor_id}...", end=" ")
        pos = client.read_param(motor_id, 'mechpos')
        print(f"✓ FOUND! Position: {pos:.3f} rad")
    except Exception:
        print("✗")

bus.shutdown()
print("\nDone.")



