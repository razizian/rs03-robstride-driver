# Setup Commands to Run Manually

All files have been created. Run these commands in order:

## 1. Install System Packages
```bash
sudo apt update
sudo apt install -y can-utils python3.12-venv git usbutils
```

## 2. Verify CH340 Detection
Plug in your CH340 USB-to-CAN adapter, then run:
```bash
lsusb | grep -i ch340
dmesg | tail -20
```
Expected: Device `1a86:7523` visible, `/dev/ttyACM*` or `/dev/ttyUSB*` appears

## 3. Install udev Rule
```bash
sudo cp udev/99-ch340-can.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Then replug the CH340 device** and verify:
```bash
ls -l /dev/ch340_can
```

## 4. Optional: Add User to dialout Group (avoids sudo for serial access)
```bash
sudo usermod -aG dialout $USER
```
**Note: You must log out and log back in for this to take effect**

## 5. Setup Python Virtual Environment
```bash
cd /home/cf-user/cursorAI/cyberfusion_sdk_rs03
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install python-dotenv robstride
```

If `robstride` is not on PyPI:
```bash
pip install python-dotenv git+https://github.com/sirwart/robstride
```

## 6. Bring Up CAN Interface
```bash
./scripts/can_up.sh
```

## 7. Test CAN Bus (Optional Smoke Test)
In one terminal:
```bash
candump -L can0 | tee logs/candump_bootstrap.log
```
Let it run for 10 seconds (Ctrl+C to stop)

In another terminal:
```bash
ip -s link show can0
```
Check that RX/TX counters increment if motor is powered and transmitting

## 8. Run Test Scripts

### Read-only test (no motion):
```bash
source .venv/bin/activate
python examples/read_params.py
```

### Motion test with logging:
```bash
candump -L can0 > logs/candump_first_motion.log &
CANDUMP_PID=$!
python examples/first_motion.py
kill $CANDUMP_PID
```

### Dry-run (connect but no motion):
```bash
python examples/first_motion.py --dry-run
```

## 9. Shutdown CAN Interface
```bash
./scripts/can_down.sh
```

## Troubleshooting Quick Reference

| Issue | Command to Fix |
|-------|---------------|
| Permission denied on /dev | `sudo usermod -aG dialout $USER` (then logout/login) |
| ModemManager interference | `sudo systemctl stop ModemManager` or `sudo systemctl disable ModemManager` |
| Interface error counters | Check termination: `ip -s link show can0` |
| Python venv creation fails | `sudo apt install python3.12-venv` |

## Created Files

```
/home/cf-user/cursorAI/cyberfusion_sdk_rs03/
├── .env                          # Configuration (PORT, BITRATE, NODE_ID)
├── .env.template                 # Template for .env
├── README.md                     # Full documentation
├── SETUP_COMMANDS.md            # This file
├── udev/
│   └── 99-ch340-can.rules       # CH340 → /dev/ch340_can
├── scripts/
│   ├── can_up.sh                # Bring up CAN (auto-detects can0/slcan0)
│   └── can_down.sh              # Shutdown CAN
├── examples/
│   ├── first_motion.py          # Motion test (±0.5 rad/s jog)
│   └── read_params.py           # Read telemetry only
└── logs/
    └── session_notes.md         # Manual logging template
```

## Next Steps After Setup

1. Test with actuator powered but no load
2. Monitor temperature during operation
3. Adjust limits in examples if needed (current: 2A, velocity: 2 rad/s)
4. Log sessions in `logs/session_notes.md`
5. Optional: Create ROS2 wrapper for robotic integration

