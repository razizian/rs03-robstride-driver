#!/usr/bin/env python3
"""
Hardware connection debugger for RS03 motor
"""
import can
import time
import threading

print("RS03 Hardware Connection Debugger")
print("=================================\n")

# Motor IDs to scan
MOTOR_IDS = [101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 127]

def scan_for_motors():
    """Scan for any responding motors"""
    print("1. Scanning for motors on CAN bus...")
    print(f"   Testing motor IDs: {MOTOR_IDS}")
    
    try:
        bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
        found_motors = []
        
        for motor_id in MOTOR_IDS:
            # Send read position command
            msg = can.Message(
                arbitration_id=motor_id,
                data=[0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                is_extended_id=False
            )
            
            print(f"\r   Scanning ID {motor_id}...", end='', flush=True)
            
            # Clear any pending messages
            while bus.recv(timeout=0):
                pass
            
            # Send command
            bus.send(msg)
            
            # Wait for response
            start_time = time.time()
            while time.time() - start_time < 0.1:  # 100ms timeout
                response = bus.recv(timeout=0.01)
                if response and response.arbitration_id == motor_id:
                    found_motors.append(motor_id)
                    print(f"\r   ✓ Found motor at ID {motor_id}!          ")
                    break
        
        print(f"\r   Scan complete.                    ")
        bus.shutdown()
        return found_motors
        
    except Exception as e:
        print(f"\n   ✗ Error: {e}")
        return []

def monitor_all_traffic():
    """Monitor all CAN traffic"""
    print("\n2. Monitoring ALL CAN traffic for 5 seconds...")
    print("   (Power cycle your motor now if you haven't already)\n")
    
    try:
        bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
        messages_seen = {}
        
        start_time = time.time()
        while time.time() - start_time < 5.0:
            msg = bus.recv(timeout=0.1)
            if msg:
                msg_id = msg.arbitration_id
                if msg_id not in messages_seen:
                    messages_seen[msg_id] = 0
                messages_seen[msg_id] += 1
                print(f"   RX: ID=0x{msg_id:03X} [{msg_id:3d}] Data={msg.data.hex()}")
        
        bus.shutdown()
        
        if messages_seen:
            print(f"\n   ✓ Saw messages from IDs: {list(messages_seen.keys())}")
        else:
            print("\n   ✗ No CAN messages received")
            
    except Exception as e:
        print(f"\n   ✗ Error: {e}")

def check_connections():
    """Check basic connectivity"""
    print("\n3. Checking connections...")
    
    # Check CAN interface
    import subprocess
    result = subprocess.run(['ip', 'link', 'show', 'can0'], capture_output=True)
    if result.returncode == 0:
        print("   ✓ CAN interface 'can0' is up")
    else:
        print("   ✗ CAN interface 'can0' not found")
        return False
    
    # Check CAN statistics
    result = subprocess.run(['ip', '-s', 'link', 'show', 'can0'], capture_output=True, text=True)
    if "TX:" in result.stdout:
        lines = result.stdout.split('\n')
        for i, line in enumerate(lines):
            if "TX:" in line and i+1 < len(lines):
                tx_stats = lines[i+1].split()
                if len(tx_stats) > 1:
                    tx_packets = int(tx_stats[1])
                    print(f"   ✓ CAN TX packets: {tx_packets}")
    
    return True

def main():
    # Check connections first
    if not check_connections():
        print("\nPlease run: sudo ./scripts/can_up.sh")
        return
    
    # Scan for motors
    print("\n" + "="*50)
    found = scan_for_motors()
    
    if found:
        print(f"\n✅ FOUND {len(found)} MOTOR(S): {found}")
        print(f"\nUse motor ID {found[0]} for testing:")
        print(f"  export TEST_MOTOR_ID={found[0]}")
        print(f"  uv run python examples/hardware_validation.py")
    else:
        print("\n⚠️  NO MOTORS FOUND")
        print("\nTrying broader monitoring...")
        monitor_all_traffic()
        
        print("\n" + "="*50)
        print("TROUBLESHOOTING CHECKLIST:")
        print("[ ] Motor powered on (12-48V DC)")
        print("[ ] MKS CANable connected to USB")
        print("[ ] CANH → CANH wiring")
        print("[ ] CANL → CANL wiring") 
        print("[ ] GND → GND wiring")
        print("[ ] 120Ω termination resistor installed")
        print("[ ] Correct bitrate (1 Mbps)")
        print("[ ] Motor ID in range 101-127")
        
        print("\nOTHER THINGS TO TRY:")
        print("1. Power cycle the motor")
        print("2. Check voltage at motor (should be 12-48V)")
        print("3. Try swapping CANH/CANL wires")
        print("4. Run: candump can0")
        print("5. Check dmesg for USB errors: dmesg | grep -i usb")

if __name__ == "__main__":
    main()

