# RobStride RS03 CAN Interface

Python SDK for controlling RobStride RS03 actuators over CAN bus using CH340 USB-to-CAN adapter.

## Quickstart

### Prerequisites
- Ubuntu with sudo access
- CH340 USB-to-CAN adapter
- RobStride RS03 actuator (no load, securely mounted)
- Power supply (12-48V DC, check actuator specs)

### Wiring
- CANH (CAN High) → CH340 CANH
- CANL (CAN Low) → CH340 CANL
- GND → Common ground
- 120Ω termination at both ends of CAN bus

### Installation Steps

#### 1. Install System Packages
```bash
sudo apt update
sudo apt install -y can-utils python3-venv git usbutils
```

#### 2. Install udev Rule
```bash
sudo cp udev/99-ch340-can.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
Replug CH340 and verify: `ls -l /dev/ch340_can`

#### 3. Setup Python Environment
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install python-dotenv robstride
```

If `robstride` is not available on PyPI:
```bash
pip install git+https://github.com/sirwart/robstride
```

#### 4. Configure Environment
```bash
cp .env.template .env
# Edit .env to set PORT, BITRATE, NODE_ID if needed
```

### Running

#### 1. Bring up CAN interface
```bash
./scripts/can_up.sh
```

#### 2. Test connection (read-only)
```bash
source .venv/bin/activate
python examples/read_params.py
```

#### 3. Run motion test
```bash
python examples/first_motion.py
```

For dry-run (no motion):
```bash
python examples/first_motion.py --dry-run
```

#### 4. Check logs
```bash
ls logs/
cat logs/session_notes.md
```

#### 5. Shutdown CAN interface
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
| CH340 not detected | Check `lsusb` for device 1a86:7523, try different USB port |
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
│   ├── can_up.sh          # Bring up CAN interface (auto-detects can0/slcan0)
│   └── can_down.sh        # Shutdown CAN interface
├── udev/
│   └── 99-ch340-can.rules # CH340 → /dev/ch340_can symlink
├── examples/
│   ├── first_motion.py    # Minimal motion test (±0.5 rad/s jog)
│   └── read_params.py     # Read telemetry (no motion)
├── logs/
│   └── session_notes.md   # Manual logging template
├── .env.template          # Configuration template
└── README.md              # This file
```

### Configuration

Edit `.env` to customize:
- `PORT`: Serial device path (default: /dev/ch340_can)
- `BITRATE`: CAN bitrate in bps (default: 1000000)
- `NODE_ID`: Motor CAN ID (default: 107)

### Next Steps

- Tune PID parameters for your application
- Implement closed-loop position/velocity control
- Use ROS2 wrapper for robotic integration (see `rs03_driver/`)
- Log telemetry to CSV/database for analysis

## ROS 2 Integration

A complete ROS 2 wrapper package is available in `rs03_driver/`. Features:
- Custom message types (MotorCommand, MotorStatus)
- Thread-safe SDK wrapper
- Multiple control modes (position/velocity/torque/operation)
- Configurable safety limits and update rates
- Launch files with parameters

See [`rs03_driver/README.md`](rs03_driver/README.md) for usage details.

