#!/usr/bin/env python3
"""Raw CAN test - bypass SDK"""
import can
import time

MOTOR_ID = 0x65  # Motor 101 in hex

print("Raw CAN Motor Test")
print("==================\n")

try:
    bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    print("✓ CAN bus connected")
    
    # Clear any pending messages
    while bus.recv(timeout=0):
        pass
    
    # Send ENABLE command (raw bytes)
    print("\n1. Sending ENABLE command...")
    enable_msg = can.Message(
        arbitration_id=MOTOR_ID,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
        is_extended_id=False
    )
    bus.send(enable_msg)
    print(f"   Sent: {enable_msg}")
    
    # Listen for response
    print("\n2. Listening for response...")
    start = time.time()
    responses = []
    
    while time.time() - start < 2.0:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == MOTOR_ID:
            responses.append(msg)
            print(f"   RX: {msg}")
    
    if responses:
        print(f"\n✅ Motor responded! Got {len(responses)} messages")
        print("\nYour motor IS working! The robstride SDK might be the issue.")
        print("Try updating robstride or using raw CAN commands.")
    else:
        print("\n❌ No response from motor")
        print("\nTry:")
        print("1. Different motor ID - might not be 101")
        print("2. Power cycle the motor")
        print("3. Check if motor is in fault/error state")
    
    # Send DISABLE command
    print("\n3. Sending DISABLE command...")
    disable_msg = can.Message(
        arbitration_id=MOTOR_ID,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
        is_extended_id=False
    )
    bus.send(disable_msg)
    
    bus.shutdown()
    
except Exception as e:
    print(f"Error: {e}")


