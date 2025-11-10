# RS03 Quick Reference Card

## 🚀 Quick Start (2 minutes)
```bash
# 1. Setup CAN
./scripts/can_up.sh

# 2. Run validation
./run_validation_suite.sh

# 3. Shutdown
./scripts/can_down.sh
```

## 🧪 Test Sequence (Recommended Order)

### 1. Hardware Check (5 min)
```bash
TEST_MOTOR_ID=101 python examples/hardware_validation.py
```
- Tests with 0.5A current limit (safe)
- Small movements only (±0.2 rad)
- Press Ctrl+C to emergency stop

### 2. Multi-Motor Test (10 min)
```bash
python examples/multi_motor_incremental.py
```
- Tests motors 101-105 progressively
- Stops at first failure
- Press Enter between stages

### 3. Performance Check (5 min)
```bash
python examples/latency_measurement.py
```
- Outputs: `latency_report.json`
- Target: <1ms average latency
- Check scaling with multiple motors

### 4. Motion Demos (10 min)
```bash
python examples/trajectory_following.py
```
- Sine waves, trapezoidal profiles
- Multi-motor coordination
- Shows tracking errors

## 🛡️ Safety Limits

| Parameter | Validation | Production |
|-----------|------------|------------|
| Current   | 0.5A       | 2.0A       |
| Velocity  | 0.5 rad/s  | 2.0 rad/s  |
| Position  | ±0.2 rad   | ±10 rad    |

## 📊 Key Metrics

**Good Performance:**
- Command latency: <1ms average
- Multi-motor scaling: <0.5ms per motor
- Position RMSE: <0.01 rad
- Command rate: >500 Hz sustained

## 🐛 Troubleshooting

| Problem | Quick Fix |
|---------|-----------|
| CAN not found | `sudo ip link show` → check for can0 |
| Permission denied | `sudo usermod -aG dialout $USER` → logout |
| Motor not responding | Check power, ID, termination resistor |
| High latency | Reduce command rate, check CPU load |
| Tracking errors | Lower gains (Kp/Kd), check mechanics |

## 📝 Environment Variables
```bash
export TEST_MOTOR_ID=101      # Motor to test
export BITRATE=1000000        # CAN bitrate
export PORT=/dev/ttyACM0      # MKS CANable device
```

## 🔧 Manual Control
```python
# Quick position command
from robstride import Client, RunMode
import can

bus = can.Bus(interface='socketcan', channel='can0')
client = Client(bus)
client.enable(101)
client.set_position(101, position=0.0, velocity=0.0, kp=50.0, kd=5.0, torque=0.0)
client.disable(101)
```

## 📚 Next Steps
- Increase limits gradually after validation
- Implement your trajectory requirements  
- Profile your specific use case
- See C++ migration plan for production

