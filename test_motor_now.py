#!/usr/bin/env python3
"""Quick test with ttyACM1"""
import os
import subprocess

# Set the correct port
os.environ['PORT'] = '/dev/ttyACM1'

print("Quick Motor Test")
print("===============")
print("✓ Found MKS CANable at /dev/ttyACM1")
print("")

# Check if CAN is up
result = subprocess.run(['ip', 'link', 'show', 'can0'], capture_output=True)
if result.returncode != 0:
    print("❌ CAN interface not set up yet!")
    print("")
    print("Please run these commands:")
    print("  sudo slcand -o -c -s8 /dev/ttyACM1 can0")
    print("  sudo ip link set can0 up")
    print("")
    print("Or use the script:")
    print("  sudo PORT=/dev/ttyACM1 ./scripts/can_up.sh")
else:
    print("✓ CAN interface is up!")
    print("")
    print("Now running hardware test...")
    # Run the hardware validation
    subprocess.run(['uv', 'run', 'python', 'examples/hardware_validation.py'])

