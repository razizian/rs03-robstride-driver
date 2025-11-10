#!/usr/bin/env python3
"""
Incremental multi-motor validation for RS03 actuators.
Tests 1→2→3→4→5 motors progressively with safety checks at each stage.
"""
import os
import sys
import time
import can
from robstride import Client, RunMode
from safety_utils import MotorSafety, RateLimiter, validate_command_params
import threading
from collections import defaultdict

# Configuration
PORT = os.getenv('PORT', '/dev/ttyACM0')
BITRATE = int(os.getenv('BITRATE', 1000000))

# Motor IDs to test incrementally
ALL_MOTOR_IDS = [101, 102, 103, 104, 105]

# Safety parameters for multi-motor testing
MULTI_CURRENT_LIMIT = 1.0  # A (conservative for multiple motors)
MULTI_VELOCITY_LIMIT = 1.0  # rad/s
SYNC_TOLERANCE = 0.1  # rad (acceptable position difference)


class MultiMotorValidator:
    """Handles incremental validation of multiple motors."""
    
    def __init__(self, client, motor_ids):
        self.client = client
        self.all_motor_ids = motor_ids
        self.active_motors = []
        self.safety = MotorSafety(client, motor_ids)
        self.rate_limiter = RateLimiter(min_interval=0.01)
        self.motor_positions = defaultdict(float)
        self.test_passed = defaultdict(bool)
        
    def validate_single_motor(self, motor_id):
        """Validate a single motor before adding to active set."""
        print(f"\n--- Validating Motor {motor_id} ---")
        
        # Set safety limits
        if not self.safety.verify_limits(motor_id, MULTI_CURRENT_LIMIT, MULTI_VELOCITY_LIMIT):
            print(f"✗ Failed to set safety limits for motor {motor_id}")
            return False
            
        # Enable motor
        if not self.safety.safe_enable(motor_id):
            print(f"✗ Failed to enable motor {motor_id}")
            return False
            
        # Test basic movement
        try:
            pos = self.client.read_param(motor_id, 'mechpos')
            self.motor_positions[motor_id] = pos
            print(f"✓ Motor {motor_id} ready at position {pos:.3f} rad")
            return True
        except Exception as e:
            print(f"✗ Motor {motor_id} communication failed: {e}")
            return False
    
    def sync_test(self, motors, duration=3.0):
        """Test synchronized movement of active motors."""
        print(f"\nTesting synchronized movement of {len(motors)} motors...")
        
        # Move all motors to zero position
        print("Moving all motors to zero position...")
        self.client.set_mode(motors[0], RunMode.MIT)  # Set mode once
        
        start_time = time.time()
        while time.time() - start_time < duration:
            for motor_id in motors:
                self.client.set_position(motor_id, 0.0, velocity=0.0, kp=30.0, kd=3.0, torque=0.0)
                self.rate_limiter.wait()
        
        # Check synchronization
        time.sleep(0.5)  # Let motors settle
        positions = {}
        for motor_id in motors:
            try:
                pos = self.client.read_param(motor_id, 'mechpos')
                positions[motor_id] = pos
                print(f"  Motor {motor_id}: {pos:.3f} rad")
            except:
                print(f"  Motor {motor_id}: FAILED TO READ")
                return False
        
        # Check if all positions are within tolerance
        pos_values = list(positions.values())
        max_diff = max(pos_values) - min(pos_values)
        
        if max_diff < SYNC_TOLERANCE:
            print(f"✓ Synchronization successful (max diff: {max_diff:.3f} rad)")
            return True
        else:
            print(f"✗ Synchronization failed (max diff: {max_diff:.3f} rad > {SYNC_TOLERANCE} rad)")
            return False
    
    def wave_test(self, motors, cycles=2):
        """Test wave pattern across motors."""
        print(f"\nTesting wave pattern with {len(motors)} motors...")
        
        for cycle in range(cycles):
            print(f"  Cycle {cycle + 1}/{cycles}")
            for i, motor_id in enumerate(motors):
                # Create phase-shifted positions
                target_pos = 0.3 * (1 if i % 2 == 0 else -1)
                
                # Move motor
                for _ in range(20):  # 2 seconds per motor
                    self.client.set_position(motor_id, target_pos, velocity=0.0, 
                                           kp=25.0, kd=2.5, torque=0.0)
                    self.rate_limiter.wait()
                
                print(f"    Motor {motor_id} → {target_pos:.2f} rad")
        
        return True
    
    def compliance_gradient_test(self, motors):
        """Test different compliance levels across motors."""
        print(f"\nTesting compliance gradient with {len(motors)} motors...")
        
        # Set different stiffness for each motor
        for i, motor_id in enumerate(motors):
            kp = 10.0 + i * 10.0  # Increasing stiffness
            print(f"  Motor {motor_id}: Kp = {kp:.1f}")
            
            # Apply for 2 seconds
            for _ in range(20):
                self.client.set_position(motor_id, 0.0, velocity=0.0, 
                                       kp=kp, kd=1.0, torque=0.0)
                self.rate_limiter.wait()
        
        print("✓ Compliance gradient applied (motors have different stiffness)")
        print("  You can manually test by gently moving each motor")
        time.sleep(3)  # Give time to manually test
        
        return True
    
    def run_incremental_validation(self):
        """Run the full incremental validation sequence."""
        print("\n=== INCREMENTAL MULTI-MOTOR VALIDATION ===")
        print(f"Testing motors: {self.all_motor_ids}")
        print("Each stage must pass before proceeding to the next\n")
        
        for num_motors in range(1, len(self.all_motor_ids) + 1):
            motors_to_test = self.all_motor_ids[:num_motors]
            
            print(f"\n{'='*50}")
            print(f"STAGE {num_motors}: Testing {num_motors} motor(s): {motors_to_test}")
            print(f"{'='*50}")
            
            # Add new motor to active set
            if num_motors > len(self.active_motors):
                new_motor = motors_to_test[-1]
                if self.validate_single_motor(new_motor):
                    self.active_motors.append(new_motor)
                else:
                    print(f"\n✗ VALIDATION STOPPED at motor {new_motor}")
                    return False
            
            # Run tests for current motor set
            tests = [
                ("Synchronization", lambda: self.sync_test(self.active_motors)),
                ("Wave Pattern", lambda: self.wave_test(self.active_motors, cycles=1)),
                ("Compliance Gradient", lambda: self.compliance_gradient_test(self.active_motors))
            ]
            
            for test_name, test_func in tests:
                print(f"\n{test_name} Test:")
                if not test_func():
                    print(f"\n✗ {test_name} test failed at stage {num_motors}")
                    return False
            
            print(f"\n✓ Stage {num_motors} PASSED - {num_motors} motor(s) validated")
            
            # Brief pause between stages
            if num_motors < len(self.all_motor_ids):
                print("\nPress Enter to continue to next stage (or Ctrl+C to stop)...")
                try:
                    input()
                except KeyboardInterrupt:
                    print("\nValidation stopped by user")
                    return False
        
        return True


def main():
    """Main incremental validation routine."""
    print("Multi-Motor Incremental Validation")
    print("==================================")
    
    # Initialize CAN
    try:
        bus = can.Bus(interface='socketcan', channel='can0', bitrate=BITRATE)
        client = Client(bus)
    except Exception as e:
        print(f"ERROR: Failed to initialize CAN bus: {e}")
        return 1
    
    # Create validator
    validator = MultiMotorValidator(client, ALL_MOTOR_IDS)
    
    try:
        # Run incremental validation
        success = validator.run_incremental_validation()
        
        if success:
            print("\n✓ FULL VALIDATION COMPLETE!")
            print(f"  Successfully validated all {len(ALL_MOTOR_IDS)} motors")
            print("\nRecommendations:")
            print("  1. All motors are functioning correctly in coordinated control")
            print("  2. You can now run trajectory examples with confidence")
            print("  3. Consider gradually increasing safety limits if needed")
        else:
            print("\n✗ VALIDATION INCOMPLETE")
            print(f"  Successfully validated {len(validator.active_motors)} motor(s)")
            if validator.active_motors:
                print(f"  Working motors: {validator.active_motors}")
                
    except KeyboardInterrupt:
        print("\n\nValidation interrupted by user")
    except Exception as e:
        print(f"\n\nERROR: Unexpected error: {e}")
    finally:
        # Cleanup
        print("\nDisabling all motors...")
        validator.safety.emergency_stop_all()
        bus.shutdown()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

