# Lessons Learned - RS03 Bring-Up

## Hardware Setup (What Actually Works)

### Adapter: MKS CANable V2.0 Pro
- **NOT** the RobStride CH340 adapter (caused communication issues)
- USB VID:PID: `16d0:117e`
- Device: `/dev/ttyACM0`
- **CRITICAL**: Set 120R switch to ON for termination
- Uses slcan interface in Linux

### Motor Configuration
- **CAN ID**: 127 (not default 107)
- Bitrate: 1 Mbps (1000000)
- Bus voltage: 44-48V measured
- Initial position: 5.884 rad

## Communication Issues Encountered

### Issue #1: Zero CAN Traffic
**Symptom**: candump showed nothing, motor didn't respond
**Cause**: Using RobStride CH340 adapter without proper termination
**Solution**: Switched to MKS CANable V2.0 Pro with 120Ω termination

### Issue #2: Motor Responds but Won't Move
**Symptom**: Can read parameters (position, velocity) but motor doesn't move
**Cause**: The robstride Python SDK only exposes enable/disable/params, NOT motion control
**Solution**: Send raw CAN frames using Control message (0x01)

## Control Modes

### Operation Control Mode (MIT Cheetah Style)
**What works**: Raw CAN frames with message type 0x01
- Position command (scaled by 10000)
- Velocity command (scaled by 100)  
- Kp gain (scaled by 100)
- Kd gain (scaled by 1000)
- Torque feedforward (scaled by 100)

**Update rate**: ~100 Hz (10 ms loop)

### What Doesn't Work
- Setting `spd_ref` parameter in Speed mode - motor ignores it
- Using `iq_ref` in Operation mode - too low-level for velocity control

## Motor Studio vs. Python SDK

Motor Studio uses:
- Operation Control mode (default)
- High-frequency control frames (0x01)
- Direct CAN communication

Python SDK (`sirwart/robstride`) only has:
- enable/disable
- read/write parameters
- **NO** motion control methods

**Bottom line**: Need to craft raw CAN frames for actual motion.

## Working Examples

1. `first_motion.py --dry-run`: Read telemetry only
2. `operation_control.py`: Velocity jog using raw CAN (THIS WORKS!)
3. `raw_control.py`: Lower-level control frame example

## Next Steps for ROS 2 Wrapper

The wrapper will need:
1. High-frequency control loop (100 Hz)
2. Raw CAN frame generation for motion commands
3. Parameter reads for telemetry feedback
4. Enable/disable lifecycle management

Can't rely on the robstride SDK for motion - need custom CAN frame builder.



