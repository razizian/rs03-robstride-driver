#!/usr/bin/env python3
"""Read motor parameters using robstride Client."""
import can
from robstride import Client
import os

NODE_ID = int(os.getenv('NODE_ID', 127))

print(f"Connecting to motor {NODE_ID} on can0...")
bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
client = Client(bus, host_can_id=0xAA)

try:
    # Read parameters
    pos = client.read_param(NODE_ID, 'mechpos')
    vel = client.read_param(NODE_ID, 'mechvel')
    vbus = client.read_param(NODE_ID, 'vbus')
    
    print(f"Position: {pos:.3f} rad")
    print(f"Velocity: {vel:.3f} rad/s")
    print(f"Bus Voltage: {vbus:.1f} V")
    print("✓ Read successful")
    
finally:
    bus.shutdown()
    print("Done.")

