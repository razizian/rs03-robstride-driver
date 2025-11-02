# ROS2 Integration Guide for RS03 Driver

## Prerequisites

1. **ROS2 Installation** (Humble, Iron, or Jazzy)
   ```bash
   # Example for Ubuntu 22.04 with ROS2 Humble
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

2. **Python Environment with UV**
   ```bash
   # From project root
   source $HOME/.local/bin/env
   uv sync --no-extras
   ```

3. **Colcon Build Tools**
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

## Build Instructions

### 1. Create ROS2 Workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Link the Package
```bash
ln -s /path/to/cyberfusion_sdk_rs03/rs03_driver .
```

### 3. Build the Package
```bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select rs03_driver
```

### 4. Source the Workspace
```bash
source install/setup.bash
```

## Running the Driver

### Single Motor Mode

1. **Start CAN Interface**
   ```bash
   cd /path/to/cyberfusion_sdk_rs03
   ./scripts/can_up.sh
   ```

2. **Activate Python Environment**
   ```bash
   source .venv/bin/activate
   ```

3. **Launch the Driver**
   ```bash
   ros2 launch rs03_driver rs03.launch.py
   ```

### Multi-Motor Mode (5 Motors)

```bash
ros2 launch rs03_driver rs03_multi.launch.py
```

This launches 5 motors with namespaces:
- `/motor_1/` (CAN ID 101)
- `/motor_2/` (CAN ID 102)
- `/motor_3/` (CAN ID 103)
- `/motor_4/` (CAN ID 104)
- `/motor_5/` (CAN ID 105)

## Testing Commands

### Single Motor Control
```bash
# Send position command
ros2 topic pub /rs03/cmd rs03_driver/msg/MotorCommand \
  "{mode: 1, position: 1.57}"

# Send MIT control command
ros2 topic pub /rs03/cmd rs03_driver/msg/MotorCommand \
  "{mode: 0, position: 0.0, velocity: 0.5, kp: 50.0, kd: 5.0, torque: 0.0}"

# Monitor status
ros2 topic echo /rs03/status
```

### Multi-Motor Control
```bash
# Run the multi-motor commander
ros2 run rs03_driver multi_motor_commander.py

# Or control individual motors
ros2 topic pub /motor_1/cmd rs03_driver/msg/MotorCommand \
  "{mode: 0, position: 0.0, velocity: 0.5, kp: 50.0, kd: 5.0, torque: 0.0}"
```

## Common Issues and Solutions

### Issue: "No module named 'robstride'"
**Solution**: Activate the virtual environment before launching
```bash
source /path/to/project/.venv/bin/activate
```

### Issue: "Failed to connect to motor"
**Solution**: 
1. Check CAN interface is up: `ip link show can0`
2. Verify motor power and wiring
3. Check motor ID matches configuration

### Issue: "Cannot import 'rs03_driver.msg'"
**Solution**: Source the workspace after building
```bash
source ~/ros2_ws/install/setup.bash
```

### Issue: Build fails with Python errors
**Solution**: Ensure Python environment matches ROS2
```bash
# Use system Python for ROS2
deactivate  # If in venv
colcon build
```

## Integration with Robot Systems

### 1. URDF Integration
Add to your robot URDF:
```xml
<joint name="motor_1_joint" type="revolute">
  <parent link="base_link"/>
  <child link="motor_1_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" effort="50" velocity="10"/>
</joint>
```

### 2. MoveIt Integration
Configure MoveIt to use the `/motor_N/cmd` topics for trajectory execution.

### 3. ros2_control Integration
Example configuration:
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  ros__parameters:
    joints:
      - motor_1_joint
      - motor_2_joint
      - motor_3_joint
      - motor_4_joint
      - motor_5_joint
    command_interfaces:
      - position
      - velocity
    state_interfaces:
      - position
      - velocity
```

## Performance Tuning

### 1. Adjust Loop Rate
```bash
ros2 launch rs03_driver rs03.launch.py loop_rate_hz:=200.0
```

### 2. Modify Safety Limits
```bash
ros2 launch rs03_driver rs03.launch.py \
  current_limit:=5.0 \
  velocity_limit:=5.0
```

### 3. CAN Bus Optimization
- Ensure proper termination (120Ω)
- Use shielded cables for long runs
- Keep CAN bus length under 40m at 1Mbps

## Debugging

### Enable Debug Output
```bash
ros2 run rs03_driver actuator_node.py --ros-args --log-level debug
```

### Monitor CAN Traffic
```bash
candump -L can0
```

### Check Node Status
```bash
ros2 node info /rs03_actuator
```

### View Parameters
```bash
ros2 param list /rs03_actuator
ros2 param get /rs03_actuator can_id
```

## Safety Considerations

1. **Always test with reduced limits first**
2. **Implement emergency stop in your application**
3. **Monitor motor temperature during operation**
4. **Use the safety utilities provided**:
   ```python
   from examples.safety_utils import MotorSafety
   ```

## Next Steps

- Review the safety audit in `docs/SAFETY_AUDIT.md`
- Test with actual hardware using reduced limits
- Integrate with your specific robot application
- Consider implementing the suggested future enhancements
