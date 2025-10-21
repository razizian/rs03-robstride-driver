#!/usr/bin/env python3
"""
ROS 2 node for RobStride RS03 actuator control.
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rs03_driver.msg import MotorCommand, MotorStatus
from rs03_driver.sdk_interface import SDKInterface
import sys


class RS03ActuatorNode(Node):
    """ROS 2 node wrapping RS03 SDK."""
    
    # Control modes (matching robstride library RunMode enum)
    MODE_OPERATION = 0  # MIT Cheetah style - hybrid control
    MODE_POSITION = 1   # Pure position control
    MODE_SPEED = 2      # Pure velocity control
    MODE_CURRENT = 3    # Pure torque/current control
    
    def __init__(self):
        super().__init__('rs03_actuator')
        
        # Declare parameters
        self.declare_parameter('can_port', '/dev/ch340_can')
        self.declare_parameter('bitrate', 1000000)
        self.declare_parameter('can_id', 107)
        self.declare_parameter('loop_rate_hz', 100.0)
        self.declare_parameter('current_limit', 2.0)
        self.declare_parameter('velocity_limit', 2.0)
        
        # Get parameters
        can_port = self.get_parameter('can_port').value
        bitrate = self.get_parameter('bitrate').value
        can_id = self.get_parameter('can_id').value
        loop_rate = self.get_parameter('loop_rate_hz').value
        self.current_limit = self.get_parameter('current_limit').value
        self.velocity_limit = self.get_parameter('velocity_limit').value
        
        self.get_logger().info(f'Initializing RS03 node: port={can_port}, id={can_id}, rate={loop_rate}Hz')
        
        # Initialize SDK interface
        self.sdk = SDKInterface(port=can_port, motor_id=can_id, bitrate=bitrate)
        
        # Connect to motor
        if not self.sdk.connect():
            self.get_logger().error('Failed to connect to motor!')
            sys.exit(1)
            
        # Set safety limits
        self.sdk.set_limits(self.current_limit, self.velocity_limit)
        
        # Enable motor
        if not self.sdk.enable():
            self.get_logger().error('Failed to enable motor!')
            sys.exit(1)
            
        self.get_logger().info('Motor enabled successfully')
        
        # Create subscribers and publishers
        self.cmd_sub = self.create_subscription(
            MotorCommand,
            'rs03/cmd',
            self.cmd_callback,
            10
        )
        
        self.status_pub = self.create_publisher(
            MotorStatus,
            'rs03/status',
            10
        )
        
        # Create timer for periodic status updates
        timer_period = 1.0 / loop_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.last_cmd = None
        
    def cmd_callback(self, msg: MotorCommand):
        """Handle incoming motor commands."""
        try:
            if msg.mode == self.MODE_OPERATION:
                self.sdk.send_operation_control(
                    position=msg.position,
                    velocity=msg.velocity,
                    kp=msg.kp,
                    kd=msg.kd,
                    torque=msg.torque
                )
                
            elif msg.mode == self.MODE_POSITION:
                self.sdk.send_position_command(msg.position)
                
            elif msg.mode == self.MODE_SPEED:
                self.sdk.send_speed_command(msg.velocity)
                
            elif msg.mode == self.MODE_CURRENT:
                self.sdk.send_current_command(msg.torque)
                
            else:
                self.get_logger().warn(f'Unknown control mode: {msg.mode}')
                
            self.last_cmd = msg
            
        except Exception as e:
            self.get_logger().error(f'Command execution failed: {e}')
            
    def timer_callback(self):
        """Periodic status publishing."""
        try:
            pos, vel, temp = self.sdk.get_telemetry()
            
            msg = MotorStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.enabled = self.sdk.is_enabled()
            msg.fault_code = 0
            msg.position = pos
            msg.velocity = vel
            msg.temperature = temp
            msg.torque = 0.0  # Not available from basic SDK
            msg.voltage = 0.0
            msg.current = 0.0
            
            self.status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Status update failed: {e}')
            
    def destroy_node(self):
        """Cleanup on shutdown."""
        self.get_logger().info('Shutting down motor...')
        self.sdk.disable()
        self.sdk.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RS03ActuatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

