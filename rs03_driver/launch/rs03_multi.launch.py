from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch 5 RS03 actuator nodes for MIT Cheetah control."""
    
    # Motor configuration for 5 nodes
    motors = [
        {'name': 'motor_1', 'can_id': 101},
        {'name': 'motor_2', 'can_id': 102},
        {'name': 'motor_3', 'can_id': 103},
        {'name': 'motor_4', 'can_id': 104},
        {'name': 'motor_5', 'can_id': 105},
    ]
    
    # Declare common launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'can_port',
            default_value='/dev/ttyACM0',
            description='CAN serial port device path'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'bitrate',
            default_value='1000000',
            description='CAN bitrate in bps'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'loop_rate_hz',
            default_value='100.0',
            description='Status publishing rate in Hz'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'current_limit',
            default_value='2.0',
            description='Current limit in Amps'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'velocity_limit',
            default_value='2.0',
            description='Velocity limit in rad/s'
        )
    )
    
    # Create nodes for each motor
    nodes = []
    for motor in motors:
        node = Node(
            package='rs03_driver',
            executable='actuator_node.py',
            name=motor['name'],
            namespace=motor['name'],
            output='screen',
            parameters=[{
                'can_port': LaunchConfiguration('can_port'),
                'bitrate': LaunchConfiguration('bitrate'),
                'can_id': motor['can_id'],
                'loop_rate_hz': LaunchConfiguration('loop_rate_hz'),
                'current_limit': LaunchConfiguration('current_limit'),
                'velocity_limit': LaunchConfiguration('velocity_limit'),
            }],
            remappings=[
                ('rs03/cmd', 'cmd'),
                ('rs03/status', 'status'),
            ]
        )
        nodes.append(node)
    
    return LaunchDescription(declared_arguments + nodes)
