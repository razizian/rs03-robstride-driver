#!/usr/bin/env python3
"""
RS03 Operation Control Mode (MIT Cheetah style).
This is the default control mode used by Motor Studio.

Control loop sends:
- position, velocity, Kp, Kd, torque at ~100 Hz

Safe starting values:
- position = 0 (no position tracking)
- velocity = desired velocity  
- Kp = 0 (no position stiffness)
- Kd = 0 (no damping)
- torque = 0 (no feedforward)
"""
import os
import time
import struct
import can

# Configuration
MOTOR_ID = int(os.getenv('NODE_ID', 127))
HOST_ID = 0xAA

def make_can_id(msg_type, host_id, motor_id):
    """Build extended CAN ID from components."""
    return (msg_type << 24) | (host_id << 8) | motor_id

def send_control(bus, motor_id, position, velocity, kp, kd, torque):
    """
    Send control frame (Operation Control mode).
    
    Args:
        position: desired position (rad), scaled by 10000
        velocity: desired velocity (rad/s), scaled by 100
        kp: position gain, scaled by 100
        kd: damping gain, scaled by 1000
        torque: feedforward torque (Nm), scaled by 100
    """
    pos_int = int(position * 10000) & 0xFFFF
    vel_int = int(velocity * 100) & 0xFFFF
    kp_int = int(kp * 100) & 0xFFFF
    kd_int = int(kd * 1000) & 0xFFFF
    torque_int = int(torque * 100) & 0xFFFF
    
    data = struct.pack('<HHHHH', pos_int, vel_int, kp_int, kd_int, torque_int)[:8]
    
    can_id = make_can_id(0x01, HOST_ID, motor_id)  # Control = 0x01
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
    bus.send(msg)

def enable_motor(bus, motor_id):
    """Enable motor."""
    can_id = make_can_id(0x03, HOST_ID, motor_id)  # Enable = 0x03
    bus.send(can.Message(arbitration_id=can_id, data=[0]*8, is_extended_id=True))
    time.sleep(0.1)

def disable_motor(bus, motor_id):
    """Disable motor."""
    can_id = make_can_id(0x04, HOST_ID, motor_id)  # Disable = 0x04
    bus.send(can.Message(arbitration_id=can_id, data=[0]*8, is_extended_id=True))

# Main demo
if __name__ == "__main__":
    print(f"Operation Control demo for motor {MOTOR_ID}")
    print("Motor will jog at ±0.5 rad/s")
    
    bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    
    try:
        enable_motor(bus, MOTOR_ID)
        print("Motor enabled. Starting motion...")
        
        # Forward jog
        print("\nJogging forward (+0.5 rad/s) for 2 seconds...")
        for i in range(20):
            send_control(bus, MOTOR_ID, 0, 0.5, 0, 0, 0)
            time.sleep(0.1)
        
        # Stop
        print("Stopping...")
        for i in range(5):
            send_control(bus, MOTOR_ID, 0, 0, 0, 0, 0)
            time.sleep(0.1)
        
        # Reverse jog
        print("Jogging reverse (-0.5 rad/s) for 2 seconds...")
        for i in range(20):
            send_control(bus, MOTOR_ID, 0, -0.5, 0, 0, 0)
            time.sleep(0.1)
        
        # Stop
        print("Stopping...")
        for i in range(5):
            send_control(bus, MOTOR_ID, 0, 0, 0, 0, 0)
            time.sleep(0.1)
        
        print("Motion complete!")
        
    finally:
        disable_motor(bus, MOTOR_ID)
        bus.shutdown()
        print("Motor disabled. Done!")



