#!/usr/bin/env python3
"""
Standalone hardware test - no dependencies required except system packages.
This is a minimal test to verify motor communication.

Requires:
- sudo apt install python3-can
"""

import subprocess
import sys
import time
import os

print("RS03 Hardware Test - Standalone Version")
print("=====================================")
print("This test requires python3-can to be installed.")
print("")

# Check if can module is available
try:
    import can
    print("✓ python3-can is installed")
except ImportError:
    print("✗ python3-can is not installed")
    print("\nPlease install with:")
    print("  sudo apt update")
    print("  sudo apt install python3-can")
    sys.exit(1)

# Check CAN interface
result = subprocess.run(['ip', 'link', 'show', 'can0'], capture_output=True, text=True)
if result.returncode != 0:
    print("\n✗ CAN interface 'can0' not found")
    print("Please run: sudo ./scripts/can_up.sh")
    sys.exit(1)
else:
    print("✓ CAN interface 'can0' is up")

# Get motor ID
motor_id = int(os.environ.get('TEST_MOTOR_ID', '101'))
print(f"\nTesting motor ID: {motor_id}")

# Basic CAN test
print("\nPerforming basic CAN test...")
print("1. Creating CAN bus connection...")

try:
    bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
    print("✓ CAN bus connected")
except Exception as e:
    print(f"✗ Failed to connect to CAN bus: {e}")
    sys.exit(1)

# Send enable command
print(f"\n2. Sending enable command to motor {motor_id}...")
# Enable command format: ID = motor_id, data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
enable_msg = can.Message(
    arbitration_id=motor_id,
    data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
    is_extended_id=False
)

try:
    bus.send(enable_msg)
    print("✓ Enable command sent")
    time.sleep(0.5)
except Exception as e:
    print(f"✗ Failed to send command: {e}")
    bus.shutdown()
    sys.exit(1)

# Listen for response
print("\n3. Listening for motor response (5 seconds)...")
start_time = time.time()
message_count = 0

try:
    while time.time() - start_time < 5.0:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == motor_id:
            message_count += 1
            print(f"   Received response from motor {motor_id}: {msg.data.hex()}")
            if message_count >= 3:  # Got enough responses
                break
except KeyboardInterrupt:
    print("\nTest interrupted by user")

# Send disable command
print(f"\n4. Sending disable command to motor {motor_id}...")
disable_msg = can.Message(
    arbitration_id=motor_id,
    data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
    is_extended_id=False
)

try:
    bus.send(disable_msg)
    print("✓ Disable command sent")
    time.sleep(0.5)
except:
    pass

bus.shutdown()

# Results
print("\n" + "="*50)
if message_count > 0:
    print(f"✓ TEST PASSED - Motor {motor_id} is responding")
    print(f"  Received {message_count} messages from motor")
    print("\nNext steps:")
    print("  1. Install full dependencies: uv sync")
    print("  2. Run complete validation: ./run_validation_suite.sh")
else:
    print(f"✗ TEST FAILED - No response from motor {motor_id}")
    print("\nPlease check:")
    print("  1. Motor power (12-48V DC)")
    print("  2. CAN wiring (CANH, CANL, GND)")
    print("  3. 120Ω termination resistors")
    print("  4. Motor ID setting")

print("")

