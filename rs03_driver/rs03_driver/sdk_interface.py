"""
Thread-safe wrapper for RobStride RS03 CAN control.
Uses raw CAN frames since robstride SDK doesn't expose motion commands.
"""
import threading
import struct
import time
from typing import Optional, Tuple
import can
from robstride import Client, RunMode


class SDKInterface:
    """Thread-safe wrapper for RS03 motor control via CAN."""
    
    HOST_ID = 0xAA
    MIN_COMMAND_INTERVAL = 0.001  # 1ms minimum between commands
    WATCHDOG_TIMEOUT = 1.0  # Stop motor if no command in 1 second
    
    def __init__(self, port: str, motor_id: int, bitrate: int):
        self.port = port
        self.motor_id = motor_id
        self.bitrate = bitrate
        self.bus: Optional[can.BusABC] = None
        self.client: Optional[Client] = None
        self._lock = threading.Lock()
        self._enabled = False
        self._last_command_time = 0
        self._watchdog_active = False
        self._watchdog_thread = None
        
    def connect(self) -> bool:
        """Connect to CAN bus and motor."""
        try:
            with self._lock:
                # Connect to CAN bus
                self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=self.bitrate)
                
                # Create client for enable/disable/params
                self.client = Client(self.bus, host_can_id=self.HOST_ID)
                
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
            
    def disconnect(self):
        """Disconnect from motor."""
        with self._lock:
            if self._enabled and self.client:
                try:
                    self.client.disable(self.motor_id)
                    self._enabled = False
                except:
                    pass
            if self.bus:
                self.bus.shutdown()
                self.bus = None
                self.client = None
                
    def enable(self) -> bool:
        """Enable motor."""
        try:
            with self._lock:
                if self.client:
                    self.client.enable(self.motor_id)
                    self._enabled = True
                    return True
            return False
        except Exception as e:
            print(f"Enable failed: {e}")
            return False
            
    def disable(self) -> bool:
        """Disable motor."""
        try:
            with self._lock:
                if self.client:
                    # Send stop command before disabling
                    try:
                        self._send_control_frame(0, 0, 10.0, 2.0, 0)
                    except:
                        pass
                    
                    self.client.disable(self.motor_id)
                    self._enabled = False
                    return True
            return False
        except Exception as e:
            print(f"Disable failed: {e}")
            return False
    
    def emergency_stop(self):
        """Emergency stop - immediately disable motor."""
        try:
            with self._lock:
                self._enabled = False
                if self.client:
                    # Try to stop motion first
                    try:
                        self._send_control_frame(0, 0, 50.0, 10.0, 0)
                    except:
                        pass
                    # Then disable
                    self.client.disable(self.motor_id)
        except:
            pass  # Best effort in emergency
            
    def set_limits(self, current_limit: float, velocity_limit: float):
        """Set and verify safety limits via parameters."""
        with self._lock:
            if self.client:
                try:
                    # Set limits
                    self.client.write_param(self.motor_id, 'limit_cur', current_limit)
                    self.client.write_param(self.motor_id, 'limit_spd', velocity_limit)
                    
                    # Verify limits were set correctly
                    actual_cur = self.client.read_param(self.motor_id, 'limit_cur')
                    actual_spd = self.client.read_param(self.motor_id, 'limit_spd')
                    
                    if abs(actual_cur - current_limit) > 0.1:
                        raise ValueError(f"Current limit verification failed: set {current_limit}, got {actual_cur}")
                    if abs(actual_spd - velocity_limit) > 0.1:
                        raise ValueError(f"Velocity limit verification failed: set {velocity_limit}, got {actual_spd}")
                        
                except Exception as e:
                    print(f"Failed to set limits: {e}")
                    raise
    
    def _make_can_id(self, msg_type: int) -> int:
        """Build extended CAN ID."""
        return (msg_type << 24) | (self.HOST_ID << 8) | self.motor_id
    
    def _send_control_frame(self, position: float, velocity: float, 
                           kp: float, kd: float, torque: float):
        """Send raw control frame (Operation Control mode) with safety validation."""
        # Safety validation - prevent dangerous commands
        if not (-10.0 <= position <= 10.0):
            raise ValueError(f"Position {position} out of safe range [-10, 10] rad")
        if not (-10.0 <= velocity <= 10.0):
            raise ValueError(f"Velocity {velocity} out of safe range [-10, 10] rad/s")
        if not (0 <= kp <= 500.0):
            raise ValueError(f"Kp {kp} out of safe range [0, 500]")
        if not (0 <= kd <= 50.0):
            raise ValueError(f"Kd {kd} out of safe range [0, 50]")
        if not (-50.0 <= torque <= 50.0):
            raise ValueError(f"Torque {torque} out of safe range [-50, 50] Nm")
        
        pos_int = int(position * 10000) & 0xFFFF
        vel_int = int(velocity * 100) & 0xFFFF
        kp_int = int(kp * 100) & 0xFFFF
        kd_int = int(kd * 1000) & 0xFFFF
        torque_int = int(torque * 100) & 0xFFFF
        
        data = struct.pack('<HHHHH', pos_int, vel_int, kp_int, kd_int, torque_int)[:8]
        
        can_id = self._make_can_id(0x01)  # Control message = 0x01
        msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
        
        if self.bus:
            self.bus.send(msg)
                
    def send_position_command(self, position: float):
        """Send position command (mode 1)."""
        with self._lock:
            if self._enabled:
                # Set run mode to Position
                if self.client:
                    self.client.write_param(self.motor_id, 'run_mode', RunMode.Position)
                    self.client.write_param(self.motor_id, 'loc_ref', position)
                
    def send_speed_command(self, velocity: float):
        """Send velocity command (mode 2)."""
        with self._lock:
            if self._enabled:
                # Use Operation Control with velocity only
                self._send_control_frame(0, velocity, 0, 0, 0)
                
    def send_current_command(self, torque: float):
        """Send torque/current command (mode 3)."""
        with self._lock:
            if self._enabled:
                # Use Operation Control with torque only
                self._send_control_frame(0, 0, 0, 0, torque)
                
    def send_operation_control(self, position: float, velocity: float, 
                               kp: float, kd: float, torque: float):
        """Send operation control command (mode 0 - MIT Cheetah style) with rate limiting."""
        with self._lock:
            if self._enabled:
                # Rate limiting
                now = time.time()
                if now - self._last_command_time < self.MIN_COMMAND_INTERVAL:
                    time.sleep(self.MIN_COMMAND_INTERVAL - (now - self._last_command_time))
                
                try:
                    self._send_control_frame(position, velocity, kp, kd, torque)
                    self._last_command_time = time.time()
                except ValueError as e:
                    print(f"Safety validation failed: {e}")
                    # Send safe stop command
                    self._send_control_frame(0, 0, 10.0, 2.0, 0)
                    raise
                
    def get_telemetry(self) -> Tuple[float, float, float]:
        """Get position, velocity, temperature."""
        with self._lock:
            if self.client:
                try:
                    pos = self.client.read_param(self.motor_id, 'mechpos')
                    vel = self.client.read_param(self.motor_id, 'mechvel')
                    # Temperature not in params, return 0
                    return (pos, vel, 0.0)
                except:
                    pass
        return (0.0, 0.0, 0.0)
        
    def is_enabled(self) -> bool:
        """Check if motor is enabled."""
        return self._enabled
