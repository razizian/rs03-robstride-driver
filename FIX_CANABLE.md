# MKS CANable Recovery Guide

## Current Issue
Your MKS CANable is in DFU (firmware update) mode:
```
STMicroelectronics STM Device in DFU Mode
```

## Quick Fix Options:

### Option 1: Simple Reset (Try First)
1. Unplug the MKS CANable from USB
2. Wait 5 seconds
3. Hold down the BOOT button on the MKS CANable (if present)
4. Plug it back in while holding BOOT
5. Release BOOT button after 2 seconds
6. Check if it appears as `/dev/ttyACM0`:
   ```bash
   ls /dev/ttyACM*
   ```

### Option 2: Power Cycle Everything
1. Turn OFF motor power supply
2. Unplug MKS CANable from USB
3. Wait 10 seconds
4. Plug MKS CANable back in first
5. Wait 2 seconds  
6. Turn ON motor power supply
7. Check device:
   ```bash
   ls /dev/ttyACM*
   lsusb
   ```

### Option 3: Different USB Port
Sometimes USB ports can be flaky:
1. Try a different USB port (preferably USB 2.0)
2. Avoid USB hubs if possible
3. Check:
   ```bash
   dmesg | tail -20
   ```

### Option 4: Manual Recovery
If it's stuck in DFU mode, you might need to:
1. Use a paper clip to press RESET button (if available)
2. Or disconnect everything for 30 seconds

## After Recovery
Once you see `/dev/ttyACM0` again:
```bash
# 1. Bring up CAN interface
sudo ./scripts/can_up.sh

# 2. Run hardware debug
uv run python debug_hardware.py

# 3. Or go straight to validation
uv run python examples/hardware_validation.py
```

## If Nothing Works
The device might need firmware re-flashing. Check:
- MKS CANable documentation
- Look for physical BOOT/RESET buttons
- Try Windows/different computer to rule out driver issues

