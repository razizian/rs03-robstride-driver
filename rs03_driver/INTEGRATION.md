# ROS 2 Integration Guide

Quick reference for integrating the RS03 driver into your ROS 2 workspace.

## Installation

### Option 1: Symlink (Development)
```bash
cd ~/ros2_ws/src
ln -s /home/cf-user/cursorAI/cyberfusion_sdk_rs03/rs03_driver .
cd ~/ros2_ws
colcon build --packages-select rs03_driver
```

### Option 2: Copy (Production)
```bash
cp -r /home/cf-user/cursorAI/cyberfusion_sdk_rs03/rs03_driver ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select rs03_driver
```

## Prerequisites

1. **CAN Interface Setup**
   - Follow parent README to configure CH340 and CAN
   - Ensure `can0` or `slcan0` is up: `ip link show can0`

2. **Python Dependencies**
   - Install robstride SDK in your ROS 2 workspace venv/system:
   ```bash
   pip install robstride
   # OR
   pip install git+https://github.com/sirwart/robstride
   ```

## Quick Test

```bash
# Terminal 1: Launch node
source ~/ros2_ws/install/setup.bash
ros2 launch rs03_driver rs03.launch.py

# Terminal 2: Send command
ros2 topic pub --once /rs03/cmd rs03_driver/msg/MotorCommand "{mode: 2, velocity: 0.5}"

# Terminal 3: Monitor status
ros2 topic echo /rs03/status
```

## Multi-Robot Setup

For multiple actuators, use namespaces:

```python
# multi_rs03.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rs03_driver',
            executable='actuator_node.py',
            name='joint1',
            namespace='robot',
            parameters=[{
                'can_id': 107,
                'can_port': '/dev/ch340_can'
            }],
            remappings=[
                ('rs03/cmd', 'joint1/cmd'),
                ('rs03/status', 'joint1/status'),
            ]
        ),
        Node(
            package='rs03_driver',
            executable='actuator_node.py',
            name='joint2',
            namespace='robot',
            parameters=[{
                'can_id': 108,
                'can_port': '/dev/ch340_can'
            }],
            remappings=[
                ('rs03/cmd', 'joint2/cmd'),
                ('rs03/status', 'joint2/status'),
            ]
        ),
    ])
```

## Parameter File Example

Create `config/rs03_params.yaml`:
```yaml
rs03_actuator:
  ros__parameters:
    can_port: "/dev/ch340_can"
    bitrate: 1000000
    can_id: 107
    loop_rate_hz: 100.0
    current_limit: 3.0
    velocity_limit: 5.0
```

Launch with:
```bash
ros2 launch rs03_driver rs03.launch.py params_file:=config/rs03_params.yaml
```

## Common Issues

### Import Error: "No module named 'robstride'"
```bash
# In your ROS 2 workspace
source install/setup.bash
pip install robstride
```

### Message Type Not Found
```bash
# Rebuild and source
cd ~/ros2_ws
colcon build --packages-select rs03_driver
source install/setup.bash
```

### Motor Not Responding
- Check CAN interface: `candump can0`
- Verify motor ID matches parameter
- Ensure motor is powered
- Check node logs: `ros2 run rs03_driver actuator_node.py`

## Integration with Other Packages

### With MoveIt2
- Create URDF with joint definitions
- Map joint names to CAN IDs
- Use trajectory controller interface (future enhancement)

### With Nav2
- Use velocity commands for mobile base
- Implement odometry publishing
- Add safety monitoring

### With Gazebo
- Create simulation plugin matching control interface
- Use same message types for sim/real consistency

## Performance Tuning

**High-rate control (200 Hz):**
```bash
ros2 launch rs03_driver rs03.launch.py loop_rate_hz:=200.0
```

**Low latency (single-threaded executor):**
```python
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(node)
executor.spin()
```

**Multi-threaded (parallel callbacks):**
```python
executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
```

## Development Tips

- Use `--dry-run` in standalone examples before ROS integration
- Monitor CAN traffic with `candump -L can0` during testing
- Log telemetry: `ros2 topic echo /rs03/status | tee motor_log.txt`
- Check message interface: `ros2 interface show rs03_driver/msg/MotorCommand`

## Next Steps

- See [README.md](README.md) for detailed API docs
- Check [../docs/ros2_wrapper_architecture.md](../docs/ros2_wrapper_architecture.md) for design details
- Review example scripts in `scripts/`

