#!/usr/bin/env python3
"""
Safety utilities for RS03 motor control.
Provides common safety functions for all examples.
"""
import time
import signal
import sys
from typing import Optional, List
from robstride import Client


class MotorSafety:
    """Safety manager for RS03 motors."""
    
    def __init__(self, client: Client, motor_ids: List[int]):
        self.client = client
        self.motor_ids = motor_ids
        self.emergency_stop_triggered = False
        
        # Install signal handlers for emergency stop
        signal.signal(signal.SIGINT, self._emergency_stop_handler)
        signal.signal(signal.SIGTERM, self._emergency_stop_handler)
    
    def _emergency_stop_handler(self, signum, frame):
        """Handle emergency stop signals."""
        print("\n!!! EMERGENCY STOP TRIGGERED !!!")
        self.emergency_stop_triggered = True
        self.emergency_stop_all()
        sys.exit(1)
    
    def verify_limits(self, motor_id: int, current_limit: float, velocity_limit: float) -> bool:
        """Set and verify safety limits for a motor."""
        try:
            # Set limits
            self.client.write_param(motor_id, 'limit_cur', current_limit)
            self.client.write_param(motor_id, 'limit_spd', velocity_limit)
            
            # Verify
            actual_cur = self.client.read_param(motor_id, 'limit_cur')
            actual_spd = self.client.read_param(motor_id, 'limit_spd')
            
            cur_ok = abs(actual_cur - current_limit) < 0.1
            spd_ok = abs(actual_spd - velocity_limit) < 0.1
            
            if not cur_ok:
                print(f"WARNING: Current limit mismatch for motor {motor_id}: "
                      f"requested {current_limit}A, got {actual_cur}A")
            if not spd_ok:
                print(f"WARNING: Velocity limit mismatch for motor {motor_id}: "
                      f"requested {velocity_limit}rad/s, got {actual_spd}rad/s")
            
            return cur_ok and spd_ok
            
        except Exception as e:
            print(f"ERROR: Failed to set/verify limits for motor {motor_id}: {e}")
            return False
    
    def safe_enable(self, motor_id: int) -> bool:
        """Enable motor with safety checks."""
        try:
            # Check if motor responds
            try:
                pos = self.client.read_param(motor_id, 'mechpos')
            except:
                print(f"ERROR: Motor {motor_id} not responding")
                return False
            
            # Enable
            self.client.enable(motor_id)
            time.sleep(0.1)  # Give motor time to enable
            
            # TODO: Verify enabled state if SDK provides this
            return True
            
        except Exception as e:
            print(f"ERROR: Failed to enable motor {motor_id}: {e}")
            return False
    
    def safe_disable(self, motor_id: int):
        """Disable motor with controlled stop."""
        try:
            # TODO: Send zero velocity command first if possible
            time.sleep(0.1)
            
            # Disable
            self.client.disable(motor_id)
            
        except Exception as e:
            print(f"WARNING: Error disabling motor {motor_id}: {e}")
    
    def emergency_stop_all(self):
        """Emergency stop all motors."""
        print("Executing emergency stop for all motors...")
        for motor_id in self.motor_ids:
            try:
                self.client.disable(motor_id)
            except:
                pass  # Best effort
    
    def check_temperature(self, motor_id: int, max_temp: float = 80.0) -> bool:
        """Check if motor temperature is safe."""
        try:
            # Note: Temperature reading might not be available in basic SDK
            # This is a placeholder for when it's available
            return True
        except:
            return True  # Assume safe if can't read
    
    def limit_acceleration(self, current_vel: float, target_vel: float, 
                          max_accel: float, dt: float) -> float:
        """Limit acceleration to prevent mechanical shock."""
        max_delta = max_accel * dt
        delta = target_vel - current_vel
        
        if delta > max_delta:
            return current_vel + max_delta
        elif delta < -max_delta:
            return current_vel - max_delta
        else:
            return target_vel


class RateLimiter:
    """Rate limiter for command sending."""
    
    def __init__(self, min_interval: float = 0.001):
        self.min_interval = min_interval
        self.last_time = 0
    
    def wait(self):
        """Wait if necessary to maintain rate limit."""
        now = time.time()
        elapsed = now - self.last_time
        
        if elapsed < self.min_interval:
            time.sleep(self.min_interval - elapsed)
        
        self.last_time = time.time()


def validate_command_params(position: float = None, velocity: float = None,
                          kp: float = None, kd: float = None, 
                          torque: float = None) -> bool:
    """Validate command parameters are within safe ranges."""
    if position is not None and not (-10.0 <= position <= 10.0):
        print(f"ERROR: Position {position} out of range [-10, 10] rad")
        return False
    
    if velocity is not None and not (-10.0 <= velocity <= 10.0):
        print(f"ERROR: Velocity {velocity} out of range [-10, 10] rad/s")
        return False
    
    if kp is not None and not (0 <= kp <= 500.0):
        print(f"ERROR: Kp {kp} out of range [0, 500]")
        return False
    
    if kd is not None and not (0 <= kd <= 50.0):
        print(f"ERROR: Kd {kd} out of range [0, 50]")
        return False
    
    if torque is not None and not (-50.0 <= torque <= 50.0):
        print(f"ERROR: Torque {torque} out of range [-50, 50] Nm")
        return False
    
    return True
