from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch RS03 actuator node with configurable parameters."""
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'can_port',
            default_value='/dev/ch340_can',
            description='CAN serial port device path'
        ),
        
        DeclareLaunchArgument(
            'bitrate',
            default_value='1000000',
            description='CAN bitrate in bps'
        ),
        
        DeclareLaunchArgument(
            'can_id',
            default_value='107',
            description='Motor CAN ID'
        ),
        
        DeclareLaunchArgument(
            'loop_rate_hz',
            default_value='100.0',
            description='Status publishing rate in Hz'
        ),
        
        DeclareLaunchArgument(
            'current_limit',
            default_value='2.0',
            description='Current limit in Amps'
        ),
        
        DeclareLaunchArgument(
            'velocity_limit',
            default_value='2.0',
            description='Velocity limit in rad/s'
        ),
        
        # Launch node
        Node(
            package='rs03_driver',
            executable='actuator_node.py',
            name='rs03_actuator',
            output='screen',
            parameters=[{
                'can_port': LaunchConfiguration('can_port'),
                'bitrate': LaunchConfiguration('bitrate'),
                'can_id': LaunchConfiguration('can_id'),
                'loop_rate_hz': LaunchConfiguration('loop_rate_hz'),
                'current_limit': LaunchConfiguration('current_limit'),
                'velocity_limit': LaunchConfiguration('velocity_limit'),
            }]
        ),
    ])

