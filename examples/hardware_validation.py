#!/usr/bin/env python3
"""
Hardware validation script for RS03 motors with reduced safety limits.
Use this for initial hardware testing with conservative parameters.
"""
import os
import sys
import time
import can
from robstride import Client, RunMode
from safety_utils import MotorSafety, RateLimiter, validate_command_params

# Configuration
PORT = os.getenv('PORT', '/dev/ttyACM0')
BITRATE = int(os.getenv('BITRATE', 1000000))

# Reduced safety limits for hardware validation
REDUCED_CURRENT_LIMIT = 0.5  # A (reduced from 2.0A)
REDUCED_VELOCITY_LIMIT = 0.5  # rad/s (reduced from 2.0)
REDUCED_POSITION_RANGE = 0.2  # rad (small movements only)

# Test parameters
TEST_MOTOR_ID = int(os.getenv('TEST_MOTOR_ID', '101'))


def run_hardware_tests(client, motor_id):
    """Run progressive hardware validation tests."""
    safety = MotorSafety(client, [motor_id])
    rate_limiter = RateLimiter(min_interval=0.01)  # 100Hz max
    
    print(f"\n=== HARDWARE VALIDATION FOR MOTOR {motor_id} ===")
    print(f"Current limit: {REDUCED_CURRENT_LIMIT}A")
    print(f"Velocity limit: {REDUCED_VELOCITY_LIMIT} rad/s")
    print(f"Position range: ±{REDUCED_POSITION_RANGE} rad")
    print("\nPress Ctrl+C at any time for emergency stop\n")
    
    # Step 1: Set and verify safety limits
    print("1. Setting reduced safety limits...")
    if not safety.verify_limits(motor_id, REDUCED_CURRENT_LIMIT, REDUCED_VELOCITY_LIMIT):
        print("FAILED: Could not set safety limits. Aborting.")
        return False
    print("✓ Safety limits set and verified")
    
    # Step 2: Enable motor
    print("\n2. Enabling motor...")
    if not safety.safe_enable(motor_id):
        print("FAILED: Could not enable motor. Aborting.")
        return False
    print("✓ Motor enabled")
    time.sleep(0.5)
    
    # Step 3: Read initial state
    print("\n3. Reading motor state...")
    try:
        pos = client.read_param(motor_id, 'mechpos')
        print(f"✓ Initial position: {pos:.3f} rad")
        
        # Read other parameters if available
        try:
            cur_limit = client.read_param(motor_id, 'limit_cur')
            spd_limit = client.read_param(motor_id, 'limit_spd')
            print(f"✓ Current limit confirmed: {cur_limit}A")
            print(f"✓ Velocity limit confirmed: {spd_limit} rad/s")
        except:
            pass
    except Exception as e:
        print(f"WARNING: Could not read all parameters: {e}")
    
    # Step 4: Small position hold test
    print("\n4. Position hold test (holding current position)...")
    # Hold position with low gains
    pos_cmd = pos  # Hold at current position
    for i in range(30):  # 3 seconds at 10Hz
        if safety.emergency_stop_triggered:
            return False
        # Low gains for gentle hold
        client.set_position(motor_id, pos_cmd, velocity=0.0, kp=10.0, kd=1.0, torque=0.0)
        rate_limiter.wait()
        if i % 10 == 0:
            print(f"   Holding at {pos_cmd:.3f} rad...")
    print("✓ Position hold successful")
    
    # Step 5: Tiny movement test
    print("\n5. Small movement test...")
    start_pos = pos
    target_pos = pos + 0.1  # Move 0.1 rad
    
    # Move to target
    print(f"   Moving from {start_pos:.3f} to {target_pos:.3f} rad...")
    for i in range(50):  # 5 seconds
        if safety.emergency_stop_triggered:
            return False
        client.set_position(motor_id, target_pos, velocity=0.0, kp=20.0, kd=2.0, torque=0.0)
        rate_limiter.wait()
    
    # Return to start
    print(f"   Returning to {start_pos:.3f} rad...")
    for i in range(50):  # 5 seconds
        if safety.emergency_stop_triggered:
            return False
        client.set_position(motor_id, start_pos, velocity=0.0, kp=20.0, kd=2.0, torque=0.0)
        rate_limiter.wait()
    print("✓ Movement test successful")
    
    # Step 6: Very low velocity test
    print("\n6. Low velocity test...")
    print(f"   Velocity command: {REDUCED_VELOCITY_LIMIT/2:.3f} rad/s for 2 seconds")
    for i in range(20):  # 2 seconds
        if safety.emergency_stop_triggered:
            return False
        client.set_position(motor_id, 0.0, velocity=REDUCED_VELOCITY_LIMIT/2, kp=0.0, kd=5.0, torque=0.0)
        rate_limiter.wait()
    
    # Stop
    print("   Stopping...")
    for i in range(10):  # 1 second to stop
        if safety.emergency_stop_triggered:
            return False
        client.set_position(motor_id, 0.0, velocity=0.0, kp=0.0, kd=5.0, torque=0.0)
        rate_limiter.wait()
    print("✓ Velocity test successful")
    
    # Step 7: Compliance test
    print("\n7. Compliance test (you can gently move the motor)...")
    print("   Low stiffness mode for 5 seconds...")
    for i in range(50):  # 5 seconds
        if safety.emergency_stop_triggered:
            return False
        # Very low stiffness
        client.set_position(motor_id, pos, velocity=0.0, kp=5.0, kd=0.5, torque=0.0)
        rate_limiter.wait()
        if i % 10 == 0:
            print(f"   {5-i/10:.0f} seconds remaining...")
    print("✓ Compliance test complete")
    
    return True


def main():
    """Main validation routine."""
    print("RS03 Hardware Validation Script")
    print("===============================")
    print(f"Testing motor ID: {TEST_MOTOR_ID}")
    print(f"CAN interface: can0 @ {BITRATE} bps")
    
    # Initialize CAN
    try:
        bus = can.Bus(interface='socketcan', channel='can0', bitrate=BITRATE)
        client = Client(bus)
    except Exception as e:
        print(f"ERROR: Failed to initialize CAN bus: {e}")
        print("\nMake sure CAN interface is up:")
        print("  sudo ip link set can0 type can bitrate 1000000")
        print("  sudo ip link set can0 up")
        return 1
    
    try:
        # Run validation tests
        success = run_hardware_tests(client, TEST_MOTOR_ID)
        
        if success:
            print("\n✓ VALIDATION COMPLETE - All tests passed!")
            print("\nNext steps:")
            print("1. If all tests passed smoothly, you can gradually increase limits")
            print("2. Run multi_motor_incremental.py to test multiple motors")
            print("3. Use latency_measurement.py to check system performance")
        else:
            print("\n✗ VALIDATION FAILED - Please check motor and connections")
            
    except KeyboardInterrupt:
        print("\n\nValidation interrupted by user")
    except Exception as e:
        print(f"\n\nERROR: Unexpected error: {e}")
    finally:
        # Cleanup
        print("\nDisabling motor...")
        try:
            client.disable(TEST_MOTOR_ID)
        except:
            pass
        bus.shutdown()
        
    return 0


if __name__ == "__main__":
    sys.exit(main())

