<!-- 7b9c1609-c42f-4956-84de-454eed6b0ff3 d2c537bd-50f5-4e14-9ed6-451d56cddc9c -->
# RobStride RS03 CAN Setup Plan

## 1. Install System Packages

```bash
sudo apt update
sudo apt install -y can-utils python3-venv git usbutils
```

Verify CH340 detection:

```bash
lsusb | grep -i ch340
dmesg | tail -20
```

Expected: Device `1a86:7523` visible, `/dev/ttyACM*` or `/dev/ttyUSB*` appears

## 2. Create udev Rule for Stable Symlink

Create `udev/99-ch340-can.rules`:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ch340_can", MODE="0666"
```

Activate:

```bash
sudo cp udev/99-ch340-can.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Replug device and verify: `ls -l /dev/ch340_can`

## 3. Create CAN Bring-Up Scripts

**`scripts/can_up.sh`** (executable):

```bash
#!/bin/bash
sudo pkill slcand 2>/dev/null || true
sudo ip link delete can0 2>/dev/null || true
sudo systemctl stop ModemManager 2>/dev/null || true

# Attempt slcand path
sudo slcand -o -c -s8 /dev/ch340_can can0 && sudo ip link set can0 up
if ip link show can0 &>/dev/null; then
    echo "✓ can0 active at 1 Mbps"
    ip -details link show can0
    exit 0
fi

# Fallback: slcan_attach
echo "Trying slcan_attach..."
sudo slcan_attach -o -c -s8 /dev/ch340_can
IFACE=$(ip -br link | grep -E 'can|slcan' | grep -v DOWN | awk '{print $1}' | head -1)
if [ -n "$IFACE" ]; then
    sudo ip link set "$IFACE" up
    echo "✓ $IFACE active at 1 Mbps"
    ip -details link show "$IFACE"
else
    echo "✗ Failed to bring up CAN interface"
    exit 1
fi
```

**`scripts/can_down.sh`** (executable):

```bash
#!/bin/bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set slcan0 down 2>/dev/null || true
sudo ip link delete can0 2>/dev/null || true
sudo pkill slcand 2>/dev/null || true
echo "CAN interfaces down"
```

Make executable: `chmod +x scripts/can_up.sh scripts/can_down.sh`

## 4. CAN Bus Smoke Test

```bash
candump -L can0 | tee logs/candump_bootstrap.log &
CANDUMP_PID=$!
sleep 10
kill $CANDUMP_PID
```

Monitor errors: `watch -n1 "ip -s link show can0"`

Expected: RX/TX counters increment, no rising error frames

## 5. Python Environment Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install robstride || pip install git+https://github.com/sirwart/robstride
```

Create `.env`:

```
PORT=/dev/ch340_can
BITRATE=1000000
NODE_ID=107
```

## 6. Minimal Motion Example

**`examples/first_motion.py`**:

```python
import os
import time
from dotenv import load_dotenv
from robstride import Motor

load_dotenv()
PORT = os.getenv('PORT', '/dev/ch340_can')
BITRATE = int(os.getenv('BITRATE', 1000000))
NODE_ID = int(os.getenv('NODE_ID', 107))

motor = Motor(port=PORT, motor_id=NODE_ID, bitrate=BITRATE)
motor.connect()

# Safety limits
motor.set_current_limit(2.0)  # 2A max
motor.set_velocity_limit(2.0)  # 2 rad/s max

motor.enable()
print("Enabled. Starting motion...")

# +0.5 rad/s for 1s
motor.set_velocity(0.5)
time.sleep(1)
motor.set_velocity(0)
time.sleep(0.2)

# -0.5 rad/s for 1s
motor.set_velocity(-0.5)
time.sleep(1)
motor.set_velocity(0)

# Read telemetry
pos, vel, temp = motor.get_position(), motor.get_velocity(), motor.get_temperature()
print(f"Position: {pos:.3f} rad, Velocity: {vel:.3f} rad/s, Temp: {temp}°C")

motor.disable()
motor.disconnect()
print("Done.")
```

Run with concurrent logging:

```bash
candump -L can0 > logs/candump_first_motion.log &
CANDUMP_PID=$!
python examples/first_motion.py
kill $CANDUMP_PID
```

## 7. README.md Quickstart Section

Add section:

```markdown
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

### Steps
1. Plug CH340, verify: `ls -l /dev/ch340_can`
2. Bring up CAN: `./scripts/can_up.sh`
3. Activate venv: `source .venv/bin/activate`
4. Run motion test: `python examples/first_motion.py`
5. Check logs: `logs/candump_first_motion.log`, `logs/session_notes.md`

### Troubleshooting
| Issue | Solution |
|-------|----------|
| No frames on candump | Check wiring, bitrate (1 Mbps), and termination |
| Input/output error on slcand | Use slcan_attach path (auto-detected in script) |
| Permission denied /dev | Add user to dialout: `sudo usermod -aG dialout $USER` |
| Interface not can0 | Script auto-detects; check output for actual name |
| CRC/ERR counters rising | Verify 120Ω termination and cable quality |
```

## 8. Logging Infrastructure

**`logs/session_notes.md`** template:

```markdown
# Session Notes

## [YYYY-MM-DD HH:MM]
- **PSU Voltage/Current Limit**: ___V / ___A
- **CAN Interface**: can0 | slcan0
- **Node ID**: 107
- **Mode**: MIT | Velocity | Position
- **Limits**: I_max=2A, V_max=2rad/s, A_max=10rad/s²
- **Results**: 
 - Position: ___ rad
 - Velocity: ___ rad/s
 - Temperature: ___ °C
 - Issues: None | [describe]
```

Candump with timestamp: `candump -L -ta can0 >> logs/candump_$(date +%Y%m%d_%H%M%S).log`

## 9. Optional Validation Tools

**`examples/read_params.py`** (dry-run):

```python
from robstride import Motor
import os
from dotenv import load_dotenv

load_dotenv()
motor = Motor(port=os.getenv('PORT'), motor_id=int(os.getenv('NODE_ID')), 
              bitrate=int(os.getenv('BITRATE')))
motor.connect()
print(f"Pos: {motor.get_position():.3f} rad")
print(f"Vel: {motor.get_velocity():.3f} rad/s")
print(f"Temp: {motor.get_temperature()}°C")
motor.disconnect()
```

Add `--dry-run` flag to `first_motion.py` (skip motion commands, read-only).

## Acceptance Criteria

- ✓ `./scripts/can_up.sh` brings up can0/slcan0 without errors
- ✓ `candump` shows valid CAN frames (not error frames)
- ✓ `python examples/first_motion.py` completes with ±0.5 rad/s jog
- ✓ Telemetry printed (position, velocity, temperature)
- ✓ Logs exist: `logs/candump_first_motion.log`, `logs/session_notes.md`

---

## Agent Mode Handoff

**Execution sequence**:

1. Create all files and scripts exactly as specified above
2. Run commands sequentially:

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                - Install system packages → verify CH340 detection
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                - Apply udev rule → confirm `/dev/ch340_can` exists
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                - Execute `./scripts/can_up.sh` → capture and display interface name
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                - Run candump smoke test → validate CAN traffic (if any)
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                - Setup Python venv → install robstride (PyPI or GitHub fallback)
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                - Execute `python examples/first_motion.py` → **stop if motion fails**

3. Auto-detect CAN interface name if not `can0` (parse `ip -br link` output)
4. If `robstride` PyPI install fails, automatically retry with GitHub URL
5. On success:

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                - List produced artifacts with absolute paths
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                - Display final telemetry output
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                - Suggest next step: "Optional: Create ROS2 wrapper for `robstride_ros2_driver`"

6. On error: Surface full error output, stop execution, await user input

### To-dos

- [ ] Install can-utils, python3-venv, git, usbutils and verify CH340 detection
- [ ] Create and activate udev rule for /dev/ch340_can symlink
- [ ] Create scripts/can_up.sh and scripts/can_down.sh with auto-detection logic
- [ ] Run candump smoke test and verify CAN traffic
- [ ] Setup .venv, install robstride SDK, create .env configuration
- [ ] Create examples/first_motion.py with safety-limited motion test
- [ ] Write README.md quickstart section with troubleshooting matrix
- [ ] Create logs/session_notes.md template and configure timestamped candump logging
- [ ] Create examples/read_params.py and add --dry-run option to first_motion.py