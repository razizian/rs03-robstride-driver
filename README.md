# RobStride RS03 CAN Interface

Python SDK for controlling RobStride RS03 actuators over CAN bus using MKS CANable adapter.

## Quickstart

### Prerequisites
- Ubuntu with sudo access
- MKS CANable USB-to-CAN adapter
- RobStride RS03 actuator (no load, securely mounted)
- Power supply (12-48V DC, check actuator specs)

### Wiring
- CANH (CAN High) → MKS CANable CANH
- CANL (CAN Low) → MKS CANable CANL
- GND → Common ground
- 120Ω termination at both ends of CAN bus

### Installation Steps

#### 1. Install System Packages
```bash
sudo apt update
sudo apt install -y can-utils git usbutils
```

#### 2. Install UV Package Manager
```bash
wget -qO- https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
```

#### 2. Verify MKS CANable Detection
```bash
ls -l /dev/ttyACM0
```
The MKS CANable should appear as `/dev/ttyACM0` when plugged in.

#### 3. Setup Python Environment
```bash
uv sync --no-extras
# UV will automatically create .venv and install all dependencies
```

#### 4. Configure Environment
```bash
cp .env.template .env
# Edit .env to set PORT, BITRATE, NODE_ID if needed
```

### Running

#### Quick Validation Suite
Run the complete validation suite to test your hardware:
```bash
./run_validation_suite.sh
```
This will guide you through:
1. Hardware validation with reduced safety limits
2. Multi-motor incremental testing 
3. Latency profiling
4. Trajectory following demos

#### Manual Testing

1. **Bring up CAN interface**
```bash
./scripts/can_up.sh
```

2. **Test connection (read-only)**
```bash
source .venv/bin/activate  # UV created this
python examples/read_params.py
```

3. **Hardware validation** (start here for new setups)
```bash
python examples/hardware_validation.py
```

4. **Run motion test**
```bash
python examples/first_motion.py
```

For dry-run (no motion):
```bash
python examples/first_motion.py --dry-run
```

5. **Shutdown CAN interface**
```bash
./scripts/can_down.sh
```

### Troubleshooting

| Issue | Solution |
|-------|----------|
| No frames on candump | Check wiring, bitrate (1 Mbps), and termination |
| Input/output error on slcand | Use slcan_attach path (auto-detected in script) |
| Permission denied /dev | Add user to dialout: `sudo usermod -aG dialout $USER` (logout required) |
| Interface not can0 | Script auto-detects; check output for actual name (slcan0) |
| CRC/ERR counters rising | Verify 120Ω termination and cable quality |
| MKS CANable not detected | Check `lsusb` for device, verify it shows as /dev/ttyACM0 |
| ModemManager conflicts | Script auto-stops it; or disable: `sudo systemctl disable ModemManager` |

### CAN Bus Monitoring

Capture CAN traffic:
```bash
candump -L can0 | tee logs/candump_$(date +%Y%m%d_%H%M%S).log
```

Monitor link statistics:
```bash
ip -s link show can0
```

Watch for errors in real-time:
```bash
watch -n1 "ip -s link show can0"
```

### Safety Notes

- **Always** secure the actuator before testing
- **Never** run with load attached during initial tests
- Default safety limits: 2A current, 2 rad/s velocity
- Stop motor before switching control modes
- Monitor temperature during extended operation

### File Structure

```
.
├── scripts/
│   ├── can_up.sh                    # Bring up CAN interface
│   └── can_down.sh                  # Shutdown CAN interface
├── examples/
│   ├── first_motion.py              # Basic motion test
│   ├── read_params.py               # Read telemetry (no motion)
│   ├── hardware_validation.py       # Safe hardware testing with reduced limits
│   ├── multi_motor_incremental.py   # Progressive multi-motor validation
│   ├── latency_measurement.py       # Command latency profiling
│   ├── trajectory_following.py      # Advanced trajectory demos
│   └── safety_utils.py              # Shared safety utilities
├── docs/
│   ├── cpp_migration_plan.md        # C++ SDK roadmap
│   └── ...                          # Other documentation
├── logs/
│   └── session_notes.md             # Manual logging template
├── rs03_driver/                     # ROS2 package
├── run_validation_suite.sh          # Automated validation script
├── .env.template                    # Configuration template
└── README.md                        # This file
```

### Configuration

Edit `.env` to customize:
- `PORT`: Serial device path (default: /dev/ttyACM0)
- `BITRATE`: CAN bitrate in bps (default: 1000000)
- `NODE_ID`: Motor CAN ID (default: 107)

### Validation Tools

#### Hardware Validation (`hardware_validation.py`)
Safe first-time testing with reduced limits:
- Current limit: 0.5A (reduced from 2A)
- Velocity limit: 0.5 rad/s (reduced from 2 rad/s)
- Tests: position hold, small movements, low velocity, compliance

#### Multi-Motor Validation (`multi_motor_incremental.py`)
Progressive testing of multiple motors:
- Tests 1→2→3→4→5 motors incrementally
- Synchronization verification
- Wave patterns and compliance gradients
- Stops at first failure for safety

#### Latency Measurement (`latency_measurement.py`)
System performance profiling:
- Measures command round-trip times
- Tests single and multi-motor scaling
- Generates detailed JSON report
- Identifies performance bottlenecks

#### Trajectory Following (`trajectory_following.py`)
Advanced motion control demonstrations:
- Sine wave trajectories
- Trapezoidal velocity profiles
- Multi-motor coordination
- Real-time tracking metrics

### Next Steps

- Run `./run_validation_suite.sh` for guided testing
- Review generated `latency_report.json` for performance data
- Tune PID parameters based on your application
- Implement custom trajectories using provided examples
- See `docs/cpp_migration_plan.md` for C++ SDK roadmap
- Use ROS2 wrapper for robotic integration (see `rs03_driver/`)

## ROS 2 Integration

A complete ROS 2 wrapper package is available in `rs03_driver/`. Features:
- Custom message types (MotorCommand, MotorStatus)
- Thread-safe SDK wrapper
- Multiple control modes (position/velocity/torque/operation)
- Configurable safety limits and update rates
- Launch files with parameters

See [`rs03_driver/README.md`](rs03_driver/README.md) for usage details.

