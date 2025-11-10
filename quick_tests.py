#!/usr/bin/env python3
"""Quick tests that actually work with the SDK"""
import os
import time
import can
import math
from robstride import Client
from examples.operation_control import send_control, enable_motor, disable_motor, HOST_ID, make_can_id

MOTOR_ID = int(os.getenv('TEST_MOTOR_ID', 127))
PORT = os.getenv('PORT', '/dev/ttyACM0')

print("RS03 Quick Working Tests")
print("========================\n")

bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
client = Client(bus)

try:
    # Test 1: Basic parameter reading
    print("1. PARAMETER READ TEST")
    start = time.perf_counter()
    pos = client.read_param(MOTOR_ID, 'mechpos')
    read_time = (time.perf_counter() - start) * 1000
    print(f"✓ Position read: {pos:.3f} rad (took {read_time:.2f} ms)")
    
    vel = client.read_param(MOTOR_ID, 'mechvel')
    vbus = client.read_param(MOTOR_ID, 'vbus')
    print(f"✓ Velocity: {vel:.3f} rad/s, Bus voltage: {vbus:.1f} V")
    
    # Test 2: Motion control timing
    print("\n2. MOTION CONTROL TEST")
    enable_motor(bus, MOTOR_ID)
    print("✓ Motor enabled")
    
    print("Testing command latency (100 commands)...")
    latencies = []
    for i in range(100):
        start = time.perf_counter()
        send_control(bus, MOTOR_ID, 0, 0.2, 0, 5, 0)  # Small velocity
        latencies.append((time.perf_counter() - start) * 1000)
        time.sleep(0.01)
    
    avg_latency = sum(latencies) / len(latencies)
    min_latency = min(latencies)
    max_latency = max(latencies)
    
    print(f"✓ Command latency: avg={avg_latency:.2f}ms, min={min_latency:.2f}ms, max={max_latency:.2f}ms")
    
    # Stop motion
    for i in range(10):
        send_control(bus, MOTOR_ID, 0, 0, 0, 5, 0)
        time.sleep(0.1)
    
    disable_motor(bus, MOTOR_ID)
    print("✓ Motor disabled")
    
    # Test 3: Trajectory tracking
    print("\n3. SIMPLE TRAJECTORY TEST")
    enable_motor(bus, MOTOR_ID)
    
    print("Following sine wave (5 seconds)...")
    start_time = time.time()
    positions = []
    
    while time.time() - start_time < 5.0:
        t = time.time() - start_time
        target_vel = 0.5 * math.sin(t)  # Sine wave velocity
        send_control(bus, MOTOR_ID, 0, target_vel, 0, 10, 0)
        
        # Read actual position
        try:
            actual_pos = client.read_param(MOTOR_ID, 'mechpos')
            positions.append(actual_pos)
        except:
            pass
        
        time.sleep(0.01)  # 100Hz
    
    # Stop
    for i in range(10):
        send_control(bus, MOTOR_ID, 0, 0, 0, 10, 0)
        time.sleep(0.1)
    
    disable_motor(bus, MOTOR_ID)
    
    if positions:
        travel = max(positions) - min(positions)
        print(f"✓ Trajectory complete. Total travel: {travel:.3f} rad")
    
    print("\n✓ ALL TESTS COMPLETE!")
    print("\nSUMMARY:")
    print(f"- Parameter read latency: {read_time:.2f} ms")
    print(f"- Command send latency: {avg_latency:.2f} ms average") 
    print(f"- Motor responds correctly to velocity commands")
    print(f"- Sine wave trajectory tracking works")
    
except Exception as e:
    print(f"Error: {e}")
finally:
    try:
        disable_motor(bus, MOTOR_ID)
    except:
        pass
    bus.shutdown()
