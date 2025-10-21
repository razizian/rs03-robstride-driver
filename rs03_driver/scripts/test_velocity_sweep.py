#!/usr/bin/env python3
"""
Example: Velocity sweep test for RS03 motor.
"""
import rclpy
from rclpy.node import Node
from rs03_driver.msg import MotorCommand, MotorStatus
import time


class VelocitySweepTest(Node):
    """Test node that commands a velocity sweep and monitors status."""
    
    def __init__(self):
        super().__init__('velocity_sweep_test')
        
        self.cmd_pub = self.create_publisher(MotorCommand, 'rs03/cmd', 10)
        self.status_sub = self.create_subscription(
            MotorStatus,
            'rs03/status',
            self.status_callback,
            10
        )
        
        self.latest_status = None
        
    def status_callback(self, msg: MotorStatus):
        """Store latest status."""
        self.latest_status = msg
        
    def run_sweep(self):
        """Execute velocity sweep."""
        velocities = [0.5, 1.0, 0.5, 0.0, -0.5, -1.0, -0.5, 0.0]
        
        self.get_logger().info('Starting velocity sweep...')
        
        for vel in velocities:
            cmd = MotorCommand()
            cmd.mode = 2  # Velocity mode
            cmd.velocity = vel
            
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'Commanded: {vel:.2f} rad/s')
            
            # Wait and monitor
            time.sleep(2.0)
            
            if self.latest_status:
                self.get_logger().info(
                    f'  Actual vel: {self.latest_status.velocity:.2f} rad/s, '
                    f'Pos: {self.latest_status.position:.2f} rad, '
                    f'Temp: {self.latest_status.temperature:.1f}°C'
                )
        
        # Stop
        stop_cmd = MotorCommand()
        stop_cmd.mode = 0
        self.cmd_pub.publish(stop_cmd)
        self.get_logger().info('Sweep complete. Motor stopped.')


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySweepTest()
    
    # Give time for status subscription to initialize
    time.sleep(1.0)
    
    try:
        node.run_sweep()
        time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

