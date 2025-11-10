# RS03 Validation Test Report - Example Output

## What You Would See When Running Tests

### 🟢 Hardware Validation Output
```
$ python examples/hardware_validation.py

=== HARDWARE VALIDATION FOR MOTOR 101 ===
Current limit: 0.5A
Velocity limit: 0.5 rad/s
Position range: ±0.2 rad

Press Ctrl+C at any time for emergency stop

1. Setting reduced safety limits...
✓ Safety limits set and verified

2. Enabling motor...
✓ Motor enabled

3. Reading motor state...
✓ Initial position: 0.042 rad
✓ Current limit confirmed: 0.5A
✓ Velocity limit confirmed: 0.5 rad/s

4. Position hold test (holding current position)...
   Holding at 0.042 rad...
   Holding at 0.042 rad...
   Holding at 0.042 rad...
✓ Position hold successful

5. Small movement test...
   Moving from 0.042 to 0.142 rad...
   Returning to 0.042 rad...
✓ Movement test successful

6. Low velocity test...
   Velocity command: 0.250 rad/s for 2 seconds
   Stopping...
✓ Velocity test successful

7. Compliance test (you can gently move the motor)...
   Low stiffness mode for 5 seconds...
   5 seconds remaining...
   4 seconds remaining...
   3 seconds remaining...
   2 seconds remaining...
   1 seconds remaining...
✓ Compliance test complete

✓ VALIDATION COMPLETE - All tests passed!
```

### 📊 Latency Measurement Output
```
$ python examples/latency_measurement.py

RS03 Command Latency Measurement Tool
====================================

1. SINGLE MOTOR COMMAND PROFILING

--- Profiling Motor 101 ---

Testing MIT Position...
  Avg: 0.82 ms
  Min: 0.65 ms
  Max: 1.24 ms
  P95: 0.95 ms

Testing Read Position...
  Avg: 1.15 ms
  Min: 0.89 ms
  Max: 2.31 ms
  P95: 1.45 ms

Testing Write Param...
  Avg: 0.93 ms
  Min: 0.71 ms
  Max: 1.18 ms
  P95: 1.02 ms

2. MULTI-MOTOR SCALING TEST

Testing with 1 motor(s): [101]
  Total time for 1 motors: 1.08 ms
  Per-motor average: 1.08 ms

Testing with 2 motor(s): [101, 102]
  Total time for 2 motors: 1.86 ms
  Per-motor average: 0.93 ms

Testing with 3 motor(s): [101, 102, 103]
  Total time for 3 motors: 2.64 ms
  Per-motor average: 0.88 ms

3. CONTINUOUS STREAMING TEST
Testing sustained command rate...
  Commands sent: 4823
  Duration: 5.00 seconds
  Actual rate: 964.6 Hz
  Target rate: 1000 Hz

Report saved to latency_report.json

============================================================
LATENCY MEASUREMENT SUMMARY
============================================================

Fastest Operations:
  single_motor/101/MIT Position: 0.82 ms avg
  single_motor/101/Write Param: 0.93 ms avg

Multi-Motor Scaling:
  1 motors: 1.08 ms total (1.08 ms per motor)
  2 motors: 1.86 ms total (0.93 ms per motor)
  3 motors: 2.64 ms total (0.88 ms per motor)

Optimization Suggestions:
  1. Use batch commands when possible
  2. Minimize parameter reads during control loops
  3. Consider parallel command sending for multi-motor control
  4. Profile your specific use case for targeted optimization
```

### 🎯 Trajectory Following Output
```
$ python examples/trajectory_following.py

============================================================
DEMO 1: Sine Wave Trajectory
============================================================

Following trajectory on motor 101
  Duration: 10.0s
  Control: Kp=40.0, Kd=4.0, Kff_vel=1.0

Progress: 100%

Tracking performance:
  Position RMSE: 0.0083 rad
  Max position error: 0.0124 rad

============================================================
DEMO 2: Trapezoidal Velocity Profile
============================================================

Following trajectory on motor 101
  Duration: 3.4s
  Control: Kp=50.0, Kd=5.0, Kff_vel=1.0

Progress: 100%

Tracking performance:
  Position RMSE: 0.0065 rad
  Max position error: 0.0098 rad

============================================================
DEMO 3: Multi-Motor Phase-Shifted Wave
============================================================

Following trajectories on 3 motors
Moving all motors to zero position...
  Motor 101: 0.002 rad
  Motor 102: -0.001 rad
  Motor 103: 0.003 rad
✓ Synchronization successful (max diff: 0.004 rad)

Wave pattern executing...
Progress: 100%

Tracking performance:
  Motor 101 - Position RMSE: 0.0091 rad
  Motor 102 - Position RMSE: 0.0087 rad
  Motor 103 - Position RMSE: 0.0094 rad
```

### 📋 Multi-Motor Incremental Validation
```
$ python examples/multi_motor_incremental.py

=== INCREMENTAL MULTI-MOTOR VALIDATION ===
Testing motors: [101, 102, 103, 104, 105]
Each stage must pass before proceeding to the next

==================================================
STAGE 1: Testing 1 motor(s): [101]
==================================================

--- Validating Motor 101 ---
✓ Motor 101 ready at position 0.042 rad

Synchronization Test:
Moving all motors to zero position...
  Motor 101: 0.001 rad
✓ Synchronization successful (max diff: 0.000 rad)

Wave Pattern Test:
  Cycle 1/1
    Motor 101 → 0.30 rad

Compliance Gradient Test:
  Motor 101: Kp = 10.0
✓ Compliance gradient applied

✓ Stage 1 PASSED - 1 motor(s) validated

Press Enter to continue to next stage...

==================================================
STAGE 2: Testing 2 motor(s): [101, 102]
==================================================

--- Validating Motor 102 ---
✓ Motor 102 ready at position -0.015 rad

Synchronization Test:
Moving all motors to zero position...
  Motor 101: 0.002 rad
  Motor 102: 0.001 rad
✓ Synchronization successful (max diff: 0.001 rad)
```

## Expected Performance Metrics

### ✅ Good Results
- **Hardware Validation**: All tests pass without errors
- **Command Latency**: < 1ms average for single motor
- **Multi-Motor Scaling**: < 0.5ms additional per motor
- **Position Tracking RMSE**: < 0.01 rad
- **Synchronization**: < 0.1 rad difference between motors
- **Sustained Command Rate**: > 500 Hz

### ⚠️ Warning Signs
- Latency > 5ms: Check CAN bus load, CPU usage
- Large tracking errors: Reduce gains, check mechanics
- Failed synchronization: Check wiring, termination
- Low command rate: Optimize control loop

## JSON Report Example (latency_report.json)
```json
{
  "timestamp": "2025-11-10T15:30:45.123456",
  "configuration": {
    "bitrate": 1000000,
    "num_samples": 100,
    "warmup_samples": 10
  },
  "results": {
    "single_motor": {
      "101": {
        "MIT Position": {
          "min_ms": 0.65,
          "max_ms": 1.24,
          "avg_ms": 0.82,
          "std_ms": 0.12,
          "p50_ms": 0.79,
          "p95_ms": 0.95,
          "p99_ms": 1.18,
          "samples": 100
        },
        "Read Position": {
          "min_ms": 0.89,
          "max_ms": 2.31,
          "avg_ms": 1.15,
          "std_ms": 0.23,
          "p50_ms": 1.08,
          "p95_ms": 1.45,
          "p99_ms": 2.12,
          "samples": 100,
          "success_rate": 1.0
        }
      }
    },
    "scaling": {
      "1": {
        "avg_ms": 1.08,
        "samples": 100
      },
      "2": {
        "avg_ms": 1.86,
        "samples": 100
      },
      "3": {
        "avg_ms": 2.64,
        "samples": 100
      }
    },
    "streaming": {
      "commands": 4823,
      "duration_s": 5.0,
      "rate_hz": 964.6
    }
  }
}
```

## Next Steps After Testing
1. ✅ If all tests pass → Gradually increase safety limits
2. 📊 Review latency_report.json → Optimize bottlenecks
3. 🎯 Implement your trajectories using examples as templates
4. 🚀 Move to production with C++ SDK (see migration plan)
