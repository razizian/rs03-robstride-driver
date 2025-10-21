#!/usr/bin/env python3
"""
Raw CAN control for RS03 motor using Control message type.
"""
import os
import time
import struct
import can

MOTOR_ID = int(os.getenv('NODE_ID', 127))
HOST_ID = 0xAA

def make_can_id(msg_type, host_id, motor_id):
    """Build extended CAN ID from components."""
    return (msg_type << 24) | (host_id << 8) | motor_id

def send_control(bus, motor_id, position, velocity, kp, kd, torque):
    """Send control frame (message type 0x01)."""
    # Pack control data: pos(2), vel(2), kp(2), kd(2), torque(2) 
    # Values are scaled integers
    pos_int = int(position * 10000) & 0xFFFF
    vel_int = int(velocity * 100) & 0xFFFF
    kp_int = int(kp * 100) & 0xFFFF
    kd_int = int(kd * 1000) & 0xFFFF
    torque_int = int(torque * 100) & 0xFFFF
    
    # Try different packing orders
    data = struct.pack('<HHHHH', pos_int, vel_int, kp_int, kd_int, torque_int)[:8]
    
    can_id = make_can_id(0x01, HOST_ID, motor_id)  # Control = 0x01
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
    bus.send(msg)

print(f"Raw control test for motor {MOTOR_ID}")
bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)

# Enable motor first
enable_id = make_can_id(0x03, HOST_ID, MOTOR_ID)  # Enable = 0x03
bus.send(can.Message(arbitration_id=enable_id, data=[0]*8, is_extended_id=True))
time.sleep(0.1)

print("\nSending velocity control commands...")
try:
    # Send velocity commands
    for i in range(40):
        if i < 20:
            vel = 0.5  # Forward
        else:
            vel = -0.5  # Reverse
            
        send_control(bus, MOTOR_ID, 0, vel, 0, 0, 0)
        time.sleep(0.1)
        
        if i % 5 == 0:
            print(f"t={i*0.1:.1f}s: velocity command = {vel:.1f} rad/s")
    
    # Stop
    send_control(bus, MOTOR_ID, 0, 0, 0, 0, 0)
    print("\nStopped")
    
finally:
    # Disable motor
    disable_id = make_can_id(0x04, HOST_ID, MOTOR_ID)  # Disable = 0x04
    bus.send(can.Message(arbitration_id=disable_id, data=[0]*8, is_extended_id=True))
    bus.shutdown()
    print("Done!")


