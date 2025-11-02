# RS03 ROS 2 Driver

ROS 2 wrapper for RobStride RS03 actuators using the Python SDK.

## Features

- Custom message types (`MotorCommand`, `MotorStatus`)
- Thread-safe CAN wrapper with raw frame control
- Configurable control loop rate
- Multiple control modes: Operation (MIT Cheetah), Position, Speed, Current
- Safety limits (current, velocity)
- Real-time status feedback

## Installation

### Prerequisites
- ROS 2 (Humble/Iron/Jazzy)
- Python 3.8+
- robstride SDK installed in your workspace venv
- CAN interface configured (see parent README)

### Build

```bash
cd ~/ros2_ws/src
ln -s /home/cf-user/cursorAI/cyberfusion_sdk_rs03/rs03_driver .
cd ~/ros2_ws
colcon build --packages-select rs03_driver
source install/setup.bash
```

## Usage

### Launch Node

Default configuration:
```bash
ros2 launch rs03_driver rs03.launch.py
```

Custom parameters:
```bash
ros2 launch rs03_driver rs03.launch.py \
  can_port:=/dev/ch340_can \
  can_id:=107 \
  loop_rate_hz:=100.0 \
  current_limit:=2.0 \
  velocity_limit:=2.0
```

### Send Commands

**Operation control (recommended - MIT Cheetah style):**
```bash
ros2 topic pub /rs03/cmd rs03_driver/msg/MotorCommand \
  "{mode: 0, position: 0.0, velocity: 0.5, torque: 0.0, kp: 0.0, kd: 0.0}"
```

**Position control:**
```bash
ros2 topic pub /rs03/cmd rs03_driver/msg/MotorCommand "{mode: 1, position: 1.57}"
```

**Speed control:**
```bash
ros2 topic pub /rs03/cmd rs03_driver/msg/MotorCommand "{mode: 2, velocity: 0.5}"
```

**Current/torque control:**
```bash
ros2 topic pub /rs03/cmd rs03_driver/msg/MotorCommand "{mode: 3, torque: 0.2}"
```

### Monitor Status

```bash
ros2 topic echo /rs03/status
```

## Message Definitions

### MotorCommand
```
uint8 mode          # 0=Operation, 1=Position, 2=Speed, 3=Current
float32 position    # rad
float32 velocity    # rad/s
float32 torque      # Nm
float32 kp          # Position gain (mode 0)
float32 kd          # Velocity gain (mode 0)
```

### MotorStatus
```
std_msgs/Header header
bool enabled
uint8 fault_code
float32 position      # rad
float32 velocity      # rad/s
float32 torque        # Nm
float32 temperature   # Celsius
float32 voltage       # V
float32 current       # A
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `can_port` | string | `/dev/ch340_can` | CAN serial device path |
| `bitrate` | int | 1000000 | CAN bitrate (bps) |
| `can_id` | int | 107 | Motor CAN ID |
| `loop_rate_hz` | float | 100.0 | Status publishing rate |
| `current_limit` | float | 2.0 | Max current (A) |
| `velocity_limit` | float | 2.0 | Max velocity (rad/s) |

## Architecture

```
┌─────────────┐
│   ROS 2     │
│   Topics    │
└──────┬──────┘
       │
┌──────▼──────────┐
│ actuator_node.py│  (ROS 2 Node)
└──────┬──────────┘
       │
┌──────▼──────────┐
│ sdk_interface.py│  (Thread-safe wrapper)
└──────┬──────────┘
       │
┌──────▼──────────┐
│  robstride SDK  │
└──────┬──────────┘
       │
┌──────▼──────────┐
│   CAN Bus       │
│  (can0/slcan0)  │
└─────────────────┘
```

## Control Modes

Modes match the robstride library RunMode enum:

0. **OPERATION (0)**: MIT Cheetah style - hybrid control (position + velocity + torque with Kp/Kd gains)
1. **POSITION (1)**: Pure position control
2. **SPEED (2)**: Pure velocity control  
3. **CURRENT (3)**: Pure torque/current control

## Safety Notes

- Node auto-disables motor on shutdown
- Safety limits enforced at SDK level
- Thread-safe command execution
- Always test with low limits first

## Troubleshooting

### Node fails to start
- Ensure CAN interface is up: `ip link show can0`
- Check motor ID matches parameter
- Verify motor is powered

### No status messages
- Check topic: `ros2 topic list`
- Verify loop rate parameter
- Check node logs: `ros2 run rs03_driver actuator_node.py`

### Commands ignored
- Ensure motor is enabled (check status)
- Verify command mode is valid (0-4)
- Check for SDK errors in node output

## Example: Velocity Sweep

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rs03_driver.msg import MotorCommand
import time

class VelocitySweep(Node):
    def __init__(self):
        super().__init__('velocity_sweep')
        self.pub = self.create_publisher(MotorCommand, 'rs03/cmd', 10)
        
    def run(self):
        cmd = MotorCommand()
        cmd.mode = 2  # Speed mode
        
        for vel in [0.5, 1.0, 0.5, 0.0, -0.5, -1.0, -0.5, 0.0]:
            cmd.velocity = vel
            self.pub.publish(cmd)
            self.get_logger().info(f'Commanded: {vel} rad/s')
            time.sleep(2.0)

def main():
    rclpy.init()
    node = VelocitySweep()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Multi-Motor Support

### Launch 5 Motors for MIT Control

```bash
ros2 launch rs03_driver rs03_multi.launch.py
```

This launches 5 motor nodes with namespaces:
- `/motor_1/` (CAN ID 101)
- `/motor_2/` (CAN ID 102)  
- `/motor_3/` (CAN ID 103)
- `/motor_4/` (CAN ID 104)
- `/motor_5/` (CAN ID 105)

### Control All Motors

```bash
ros2 run rs03_driver multi_motor_commander.py
```

### Monitor All Motors

```bash
# In separate terminals:
ros2 topic echo /motor_1/status
ros2 topic echo /motor_2/status
# etc...
```

### Multi-Motor Configuration

See `config/rs03_multi_params.yaml` for multi-motor parameter configuration.

## Future Enhancements

- [x] Multi-actuator support with namespacing
- [ ] Diagnostics topic with detailed fault info
- [ ] Action server for trajectory execution
- [ ] Service interface for parameter changes
- [x] Launch file for multi-joint systems
- [ ] URDF integration examples

## License

MIT

