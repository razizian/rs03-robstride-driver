#!/usr/bin/env python3
"""
Multi-motor MIT Cheetah control demo for 5 RS03 actuators.
Demonstrates coordinated control of multiple motors.
"""
import os
import sys
import time
import can
from robstride import Client, RunMode
import threading

# Configuration
PORT = os.getenv('PORT', '/dev/ttyACM0')
BITRATE = int(os.getenv('BITRATE', 1000000))

# Motor IDs for 5 actuators
MOTOR_IDS = [101, 102, 103, 104, 105]

# Safety limits
CURRENT_LIMIT = 2.0  # A
VELOCITY_LIMIT = 2.0  # rad/s

def init_motor(client, motor_id):
    """Initialize a single motor with safety limits."""
    try:
        print(f"Initializing motor {motor_id}...")
        
        # Set safety limits
        client.write_param(motor_id, 'limit_cur', CURRENT_LIMIT)
        client.write_param(motor_id, 'limit_spd', VELOCITY_LIMIT)
        
        # Enable motor
        client.enable(motor_id)
        print(f"Motor {motor_id} enabled successfully")
        
        # Read initial position
        pos = client.read_param(motor_id, 'mechpos')
        print(f"Motor {motor_id} initial position: {pos:.3f} rad")
        
        return True
    except Exception as e:
        print(f"Failed to initialize motor {motor_id}: {e}")
        return False

def send_mit_command(client, motor_id, position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0):
    """Send MIT Cheetah style command to motor."""
    try:
        # Using operation control mode (mode 0)
        client.set_mode(motor_id, RunMode.OPERATION)
        
        # Send command
        # Note: The robstride SDK might not expose the full MIT command
        # This is a placeholder for the actual implementation
        print(f"Motor {motor_id}: pos={position:.2f}, vel={velocity:.2f}, kp={kp:.1f}, kd={kd:.1f}, torque={torque:.2f}")
        
    except Exception as e:
        print(f"Error sending command to motor {motor_id}: {e}")

def main():
    """Main control loop for 5 motors."""
    print(f"Connecting to motors via {PORT} at {BITRATE} bps...")
    
    # Initialize CAN bus
    bus = can.Bus(interface='socketcan', channel='can0', bitrate=BITRATE)
    client = Client(bus)
    
    # Initialize all motors
    active_motors = []
    for motor_id in MOTOR_IDS:
        if init_motor(client, motor_id):
            active_motors.append(motor_id)
    
    if not active_motors:
        print("No motors initialized successfully!")
        bus.shutdown()
        return
    
    print(f"\n{len(active_motors)} motors ready. Starting coordinated control...")
    
    try:
        # Example 1: Synchronized position hold with compliance
        print("\n--- Position Hold with Compliance ---")
        for motor_id in active_motors:
            send_mit_command(client, motor_id, 
                           position=0.0,    # Hold at 0 rad
                           velocity=0.0,    # No velocity
                           kp=50.0,         # Position gain
                           kd=5.0,          # Damping gain
                           torque=0.0)      # No feedforward torque
        time.sleep(2)
        
        # Example 2: Wave pattern - motors move in sequence
        print("\n--- Wave Pattern ---")
        for i in range(10):
            for idx, motor_id in enumerate(active_motors):
                phase = (i + idx) * 0.5
                position = 0.5 * (1 + (-1)**idx)  # Alternate +/- 0.5 rad
                send_mit_command(client, motor_id,
                               position=position,
                               velocity=0.0,
                               kp=30.0,
                               kd=3.0,
                               torque=0.0)
            time.sleep(0.5)
        
        # Example 3: Synchronized velocity control
        print("\n--- Synchronized Velocity ---")
        for motor_id in active_motors:
            send_mit_command(client, motor_id,
                           position=0.0,
                           velocity=0.5,    # 0.5 rad/s
                           kp=0.0,          # No position control
                           kd=10.0,         # Velocity damping
                           torque=0.0)
        time.sleep(3)
        
        # Example 4: Compliance test - different stiffness
        print("\n--- Variable Compliance ---")
        for idx, motor_id in enumerate(active_motors):
            kp = 10.0 + idx * 10.0  # Increasing stiffness
            send_mit_command(client, motor_id,
                           position=0.0,
                           velocity=0.0,
                           kp=kp,
                           kd=2.0,
                           torque=0.0)
            print(f"Motor {motor_id} stiffness: Kp={kp}")
        time.sleep(3)
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        # Disable all motors
        print("\nDisabling all motors...")
        for motor_id in active_motors:
            try:
                client.disable(motor_id)
                print(f"Motor {motor_id} disabled")
            except:
                pass
        
        bus.shutdown()
        print("Done.")

if __name__ == "__main__":
    main()
