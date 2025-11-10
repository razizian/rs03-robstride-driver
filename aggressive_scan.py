#!/usr/bin/env python3
"""Aggressive motor scan - try everything"""
import can
import time

print("AGGRESSIVE MOTOR SCAN")
print("=" * 50)
print("This will try ALL possible motor IDs\n")

bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)

# Try IDs from 1 to 255
print("Scanning IDs 1-255 (this takes ~1 minute)...")
found = []

for motor_id in range(1, 256):
    # Clear buffer
    while bus.recv(timeout=0):
        pass
    
    # Send enable command
    msg = can.Message(
        arbitration_id=motor_id,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
        is_extended_id=False
    )
    
    try:
        bus.send(msg)
        
        # Very short wait
        response = bus.recv(timeout=0.05)
        if response:
            found.append((motor_id, response))
            print(f"✓ Response from ID {motor_id} (0x{motor_id:02X}): {response.data.hex()}")
    except:
        pass
    
    if motor_id % 50 == 0:
        print(f"  ...checked up to ID {motor_id}")

print("\n" + "=" * 50)
if found:
    print(f"✅ FOUND {len(found)} RESPONDING DEVICE(S):")
    for motor_id, response in found:
        print(f"   ID {motor_id} (0x{motor_id:02X})")
    print(f"\nUse motor ID {found[0][0]} for testing!")
else:
    print("❌ NO DEVICES RESPONDED")
    print("\nThis means:")
    print("1. Motor is not powered")
    print("2. Motor is in fault/error state") 
    print("3. Wrong bitrate (try 500kbps)")
    print("4. Hardware issue")
    print("\nTry this:")
    print("  - Power cycle the motor completely")
    print("  - Check for error LEDs on motor")
    print("  - Try different bitrate:")
    print("    sudo ip link set can0 down")
    print("    sudo ip link set can0 type can bitrate 500000")
    print("    sudo ip link set can0 up")

bus.shutdown()

