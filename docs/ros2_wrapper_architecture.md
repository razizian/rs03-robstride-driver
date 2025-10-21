# ROS 2 Wrapper Architecture (v1.0 - Implemented)

## Goal
Expose RS03-EN via a ROS 2 node using the Python SDK:
- Accept cmd messages (position/velocity/torque)
- Publish status (pos, vel, temp, faults)
- Parameterize CAN ID, bitrate, update rate

## Package Layout (ament_cmake)
```
rs03_driver/
 ├─ package.xml              # ROS 2 package manifest
 ├─ CMakeLists.txt           # Build configuration with message generation
 ├─ README.md                # Package documentation
 ├─ msg/
 │   ├─ MotorCommand.msg     # Command message (mode, targets, gains)
 │   └─ MotorStatus.msg      # Status message (state, telemetry)
 ├─ launch/
 │   └─ rs03.launch.py       # Launch file with parameters
 ├─ scripts/
 │   ├─ actuator_node.py     # Main ROS 2 node (executable)
 │   └─ test_velocity_sweep.py  # Example test script
 └─ rs03_driver/
     ├─ __init__.py
     └─ sdk_interface.py     # Thread-safe SDK wrapper
```

## Node I/O
- **Subscriptions:**
  - `/rs03/cmd` (MotorCommand) - Control commands
  
- **Publications:**
  - `/rs03/status` (MotorStatus) - Real-time feedback at loop_rate_hz

- **Parameters:**
  - `can_port` (string, default: `/dev/ch340_can`) - CAN serial device
  - `bitrate` (int, default: 1000000) - CAN bitrate
  - `can_id` (int, default: 107) - Motor ID
  - `loop_rate_hz` (float, default: 100.0) - Status publishing rate
  - `current_limit` (float, default: 2.0) - Max current (A)
  - `velocity_limit` (float, default: 2.0) - Max velocity (rad/s)

## Message Definitions

### MotorCommand.msg
```
uint8 mode              # 0=Operation, 1=Position, 2=Speed, 3=Current
float32 position        # Target position (rad)
float32 velocity        # Target velocity (rad/s)
float32 torque          # Target torque (Nm)
float32 kp              # Position gain (operation mode)
float32 kd              # Velocity gain (operation mode)
```

### MotorStatus.msg
```
std_msgs/Header header
bool enabled            # Motor enable state
uint8 fault_code        # 0=OK, non-zero=fault
float32 position        # Current position (rad)
float32 velocity        # Current velocity (rad/s)
float32 torque          # Current torque (Nm)
float32 temperature     # Temperature (°C)
float32 voltage         # Bus voltage (V)
float32 current         # Motor current (A)
```

## Control Modes

Aligned with robstride library RunMode enum:

| Mode | Value | Description |
|------|-------|-------------|
| OPERATION | 0 | MIT Cheetah style - hybrid control (pos+vel+torque with gains) |
| POSITION | 1 | Pure position control |
| SPEED | 2 | Pure velocity control |
| CURRENT | 3 | Pure torque/current control |

## Control Loop Architecture

```
Start → Connect SDK → Set Limits → Enable Motor
                                      ↓
          ┌───────────────────────────┴─────────────────────┐
          ↓                                                   ↓
    Timer Callback (loop_rate_hz)              Command Callback (async)
          ↓                                                   ↓
    Read Telemetry                              Parse mode & execute:
          ↓                                       - MODE_OPERATION → send_op_control()
    Publish /rs03/status                         - MODE_POSITION → set_position()
          ↑                                       - MODE_SPEED → send_control(vel)
          └───────────────────────────────────────- MODE_CURRENT → send_control(torque)
Shutdown → Disable Motor → Disconnect
```

## SDK Interface Layer

The `sdk_interface.py` provides thread-safe CAN control:
- Mutex-protected CAN bus access
- Raw CAN frame construction (robstride SDK doesn't expose motion commands)
- State management (enabled/disabled)
- Safety limit enforcement via parameters
- Clean error handling

**Key Methods:**
- `connect()` / `disconnect()`
- `enable()` / `disable()` (via robstride.Client)
- `set_limits(current, velocity)` (via parameters)
- `send_position_command(pos)` (writes run_mode param)
- `send_speed_command(vel)` (raw CAN control frame)
- `send_current_command(torque)` (raw CAN control frame)
- `send_operation_control(pos, vel, kp, kd, torque)` (raw CAN control frame)
- `get_telemetry() -> (pos, vel, temp)` (via parameters)

## Usage Examples

### Launch Node
```bash
ros2 launch rs03_driver rs03.launch.py can_id:=107 loop_rate_hz:=100.0
```

### Send Speed Command
```bash
ros2 topic pub /rs03/cmd rs03_driver/msg/MotorCommand "{mode: 2, velocity: 0.5}"
```

### Send Operation Control Command
```bash
ros2 topic pub /rs03/cmd rs03_driver/msg/MotorCommand \
  "{mode: 0, position: 0.0, velocity: 0.5, kp: 0.0, kd: 0.0, torque: 0.0}"
```

### Monitor Status
```bash
ros2 topic echo /rs03/status
```

### Run Test Sweep
```bash
ros2 run rs03_driver test_velocity_sweep.py
```

## Safety Features
- Automatic motor disable on node shutdown
- Current/velocity limits enforced at SDK level
- Thread-safe command execution
- Graceful error handling with logging

## Future Enhancements
- Multi-actuator: namespace per joint
- Diagnostics topic with detailed fault codes
- Action server for trajectory execution
- Service interface for runtime parameter changes
- Joint trajectory controller interface
- URDF/xacro integration examples

## Build & Install

```bash
# In ROS 2 workspace
cd ~/ros2_ws/src
ln -s /path/to/rs03_driver .
cd ~/ros2_ws
colcon build --packages-select rs03_driver
source install/setup.bash
```

## Dependencies
- ROS 2 (Humble/Iron/Jazzy)
- rclpy
- std_msgs
- geometry_msgs (future use)
- robstride Python SDK
- CAN interface (can0/slcan0)

## Status: ⚠️ Implemented with Raw CAN

Core features implemented using raw CAN frames. The robstride Python SDK only exposes enable/disable/parameter access, not motion commands. Motion control uses direct CAN frame construction based on working examples.

**Working:**
- Message definitions
- Node structure and ROS 2 integration
- Thread-safe CAN wrapper
- Operation control mode (MIT Cheetah style)
- Parameter-based position control

**Requires Testing:**
- Full integration with ROS 2 environment
- All control modes under load
- Multi-motor scenarios

