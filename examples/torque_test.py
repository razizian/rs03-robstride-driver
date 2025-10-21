#!/usr/bin/env python3
"""
Torque control test using current reference.
"""
import os
import time
import can
from robstride import Client, RunMode

MOTOR_ID = int(os.getenv('NODE_ID', 127))

print(f"Torque control test for motor {MOTOR_ID}")
bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
client = Client(bus)

try:
    # Set limits
    client.write_param(MOTOR_ID, 'limit_cur', 2.0)
    
    # Enable in Operation mode
    client.enable(MOTOR_ID)
    client.write_param(MOTOR_ID, 'run_mode', RunMode.Operation)
    
    print("\nApplying small torque for 2 seconds...")
    client.write_param(MOTOR_ID, 'iq_ref', 0.5)  # 0.5A current
    
    for i in range(4):
        time.sleep(0.5)
        pos = client.read_param(MOTOR_ID, 'mechpos')
        vel = client.read_param(MOTOR_ID, 'mechvel')
        iq = client.read_param(MOTOR_ID, 'iqf')  # actual current
        print(f"  t={i*0.5:.1f}s: pos={pos:.3f} rad, vel={vel:.3f} rad/s, iq={iq:.3f} A")
    
    # Stop
    client.write_param(MOTOR_ID, 'iq_ref', 0.0)
    
finally:
    client.disable(MOTOR_ID)
    bus.shutdown()
    print("\nDone!")


