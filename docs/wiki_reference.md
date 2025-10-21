# Quick Reference (Working Config)

## Hardware
- Adapter: MKS CANable V2.0 Pro (VID:PID 16d0:117e)
- Device: /dev/ttyACM0
- Motor: RobStride RS03-EN
- CAN ID: **127** (verified working)
- Bitrate: 1,000,000 bps
- Bus Voltage: 44.5V measured
- **120Ω termination**: Enabled on MKS adapter

## Software
- Python SDK: `git+https://github.com/sirwart/robstride`
- Control: Raw CAN frames (Operation Control 0x01)
- Update rate: 100 Hz recommended

## Key Commands
```bash
# Bring up CAN
sudo slcand -o -c -s8 /dev/ttyACM0 can0
sudo ip link set can0 up

# Test (read-only)
NODE_ID=127 python examples/first_motion.py --dry-run

# Motion
NODE_ID=127 python examples/operation_control.py

# Monitor
candump -ta can0
```

## Limits (Safe Defaults)
- I_max: 2 A
- V_max: 2 rad/s (0.5 rad/s tested)
- Position: No limits set yet

## Files
- Logs: `logs/candump_*.log`
- Examples: `examples/operation_control.py` (working!)
- Docs: `docs/LESSONS_LEARNED.md`

