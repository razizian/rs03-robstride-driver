# RS03-EN Bring-Up (Ubuntu + MKS CANable)

## Prereqs
- Ubuntu 22.04+, sudo
- MKS CANable V2.0 Pro (or compatible gs_usb adapter)
- RS03-EN actuator + 24–60V DC PSU (nominal 48V)
- python3, can-utils
- **CRITICAL**: 120Ω termination enabled on MKS adapter (120R switch)

## Install
```bash
sudo apt update && sudo apt install -y can-utils python3-venv git usbutils
lsusb | grep -i "16d0:117e"  # MKS CANable
# Should see /dev/ttyACM0
```

## Bring up CAN
```bash
# MKS CANable (slcan mode)
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up
ip link show can0  # verify UP
```

## Test connection (read-only)
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install --upgrade pip pyserial python-dotenv
pip install git+https://github.com/sirwart/robstride
NODE_ID=127 python examples/first_motion.py --dry-run
```

## Minimal motion (Operation Control)
```bash
source .venv/bin/activate
NODE_ID=127 python examples/operation_control.py
# Motor will jog ±0.5 rad/s
```

## Troubleshooting
| Symptom                 | Fix                                      |
|-------------------------|-------------------------------------------|
| No can0                 | Replug; use `slcan_attach` fallback       |
| ERR counters rising     | Add/verify 120Ω termination both ends     |
| Permission denied /dev  | `sudo usermod -aG dialout $USER`          |
| No frames on candump    | Check wiring, 1 Mbps, CANH/CANL polarity  |

