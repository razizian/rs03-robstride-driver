#!/usr/bin/env python3
"""
ROS 2 multi-motor commander for 5-node MIT control.
Publishes coordinated commands to all 5 motors.
"""
import rclpy
from rclpy.node import Node
from rs03_driver.msg import MotorCommand
import math
import time


class MultiMotorCommander(Node):
    """Coordinates commands for 5 RS03 motors."""
    
    def __init__(self):
        super().__init__('multi_motor_commander')
        
        # Motor namespaces
        self.motor_names = ['motor_1', 'motor_2', 'motor_3', 'motor_4', 'motor_5']
        
        # Create publishers for each motor
        self.publishers = {}
        for name in self.motor_names:
            topic = f'/{name}/cmd'
            self.publishers[name] = self.create_publisher(
                MotorCommand, 
                topic, 
                10
            )
            self.get_logger().info(f'Created publisher for {topic}')
        
        # Timer for periodic commands
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz
        self.start_time = time.time()
        
    def timer_callback(self):
        """Send coordinated commands to all motors."""
        current_time = time.time() - self.start_time
        
        # Example: Sinusoidal position commands with phase offset
        for idx, name in enumerate(self.motor_names):
            msg = MotorCommand()
            
            # MIT Cheetah mode (Operation mode = 0)
            msg.mode = 0
            
            # Phase-shifted sinusoidal position
            phase = idx * (2 * math.pi / len(self.motor_names))
            msg.position = 0.5 * math.sin(0.5 * current_time + phase)
            
            # Zero velocity command (let position control handle it)
            msg.velocity = 0.0
            
            # MIT control gains
            msg.kp = 50.0  # Position gain
            msg.kd = 5.0   # Velocity gain
            
            # No feedforward torque
            msg.torque = 0.0
            
            # Publish command
            self.publishers[name].publish(msg)
        
        # Log every second
        if int(current_time) > int(current_time - 0.02):
            self.get_logger().info(f'Time: {current_time:.1f}s - Sending commands to all motors')
    
    def send_synchronized_command(self, position=0.0, velocity=0.0, kp=50.0, kd=5.0, torque=0.0):
        """Send the same command to all motors."""
        msg = MotorCommand()
        msg.mode = 0  # MIT mode
        msg.position = position
        msg.velocity = velocity
        msg.kp = kp
        msg.kd = kd
        msg.torque = torque
        
        for name in self.motor_names:
            self.publishers[name].publish(msg)
    
    def send_wave_pattern(self):
        """Send wave pattern commands."""
        current_time = time.time() - self.start_time
        
        for idx, name in enumerate(self.motor_names):
            msg = MotorCommand()
            msg.mode = 0
            
            # Create wave effect
            delay = idx * 0.2  # 200ms delay between motors
            msg.position = 0.3 * math.sin(current_time - delay)
            msg.velocity = 0.0
            msg.kp = 40.0
            msg.kd = 4.0
            msg.torque = 0.0
            
            self.publishers[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    commander = MultiMotorCommander()
    
    try:
        # Run for 30 seconds
        commander.get_logger().info('Starting multi-motor control demo...')
        rclpy.spin(commander)
        
    except KeyboardInterrupt:
        commander.get_logger().info('Shutting down...')
        
        # Send zero command to all motors
        commander.send_synchronized_command(
            position=0.0, 
            velocity=0.0, 
            kp=20.0, 
            kd=2.0, 
            torque=0.0
        )
    
    finally:
        commander.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
