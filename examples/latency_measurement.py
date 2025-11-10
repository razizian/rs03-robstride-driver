#!/usr/bin/env python3
"""
Command-to-actuation latency measurement tool for RS03 motors.
Measures round-trip times and system performance characteristics.
"""
import os
import sys
import time
import can
from robstride import Client, RunMode
import numpy as np
from datetime import datetime
import json

# Configuration
PORT = os.getenv('PORT', '/dev/ttyACM0')
BITRATE = int(os.getenv('BITRATE', 1000000))

# Test parameters
TEST_MOTOR_IDS = [int(os.getenv('TEST_MOTOR_ID', 127))]
NUM_SAMPLES = 100  # Samples per test
WARMUP_SAMPLES = 10  # Warmup iterations


class LatencyProfiler:
    """Profiles command latency for RS03 motors."""
    
    def __init__(self, client):
        self.client = client
        self.results = {}
        
    def measure_single_command_latency(self, motor_id, command_func, samples=NUM_SAMPLES):
        """Measure latency for a single command type."""
        latencies = []
        
        # Warmup
        for _ in range(WARMUP_SAMPLES):
            command_func(motor_id)
            time.sleep(0.001)
        
        # Actual measurement
        for _ in range(samples):
            start_time = time.perf_counter()
            command_func(motor_id)
            end_time = time.perf_counter()
            
            latency_ms = (end_time - start_time) * 1000
            latencies.append(latency_ms)
            
            # Small delay to prevent flooding
            time.sleep(0.001)
        
        return self._calculate_stats(latencies)
    
    def measure_multi_motor_latency(self, motor_ids, command_func, samples=NUM_SAMPLES):
        """Measure latency for commanding multiple motors."""
        latencies = []
        
        # Warmup
        for _ in range(WARMUP_SAMPLES):
            for motor_id in motor_ids:
                command_func(motor_id)
            time.sleep(0.001)
        
        # Actual measurement
        for _ in range(samples):
            start_time = time.perf_counter()
            for motor_id in motor_ids:
                command_func(motor_id)
            end_time = time.perf_counter()
            
            latency_ms = (end_time - start_time) * 1000
            latencies.append(latency_ms)
            
            time.sleep(0.001)
        
        return self._calculate_stats(latencies)
    
    def measure_read_latency(self, motor_id, param_name, samples=NUM_SAMPLES):
        """Measure latency for reading parameters."""
        latencies = []
        
        # Warmup
        for _ in range(WARMUP_SAMPLES):
            try:
                self.client.read_param(motor_id, param_name)
            except:
                pass
            time.sleep(0.001)
        
        # Actual measurement
        successful_reads = 0
        for _ in range(samples):
            start_time = time.perf_counter()
            try:
                value = self.client.read_param(motor_id, param_name)
                end_time = time.perf_counter()
                successful_reads += 1
                
                latency_ms = (end_time - start_time) * 1000
                latencies.append(latency_ms)
            except:
                pass  # Skip failed reads
            
            time.sleep(0.001)
        
        stats = self._calculate_stats(latencies) if latencies else {}
        stats['success_rate'] = successful_reads / samples
        return stats
    
    def _calculate_stats(self, latencies):
        """Calculate statistics from latency measurements."""
        if not latencies:
            return {}
            
        return {
            'min_ms': min(latencies),
            'max_ms': max(latencies),
            'avg_ms': np.mean(latencies),
            'std_ms': np.std(latencies),
            'p50_ms': np.percentile(latencies, 50),
            'p95_ms': np.percentile(latencies, 95),
            'p99_ms': np.percentile(latencies, 99),
            'samples': len(latencies)
        }
    
    def profile_motor_commands(self, motor_id):
        """Profile various command types for a single motor."""
        print(f"\n--- Profiling Motor {motor_id} ---")
        
        # Enable motor first
        try:
            self.client.enable(motor_id)
            time.sleep(0.5)
        except:
            print(f"Failed to enable motor {motor_id}")
            return
        
        # Define test commands
        test_commands = {
            'MIT Position': lambda m: self.client.set_position(m, 0.0, 0.0, 10.0, 1.0, 0.0),
            'Read Position': lambda m: self.client.read_param(m, 'mechpos'),
            'Write Param': lambda m: self.client.write_param(m, 'limit_cur', 1.0),
            'Enable/Disable': lambda m: (self.client.enable(m), self.client.disable(m))
        }
        
        motor_results = {}
        
        # Test each command type
        for cmd_name, cmd_func in test_commands.items():
            print(f"\nTesting {cmd_name}...")
            
            if cmd_name == 'Read Position':
                stats = self.measure_read_latency(motor_id, 'mechpos')
            else:
                stats = self.measure_single_command_latency(motor_id, cmd_func)
            
            motor_results[cmd_name] = stats
            
            if stats:
                print(f"  Avg: {stats.get('avg_ms', 0):.2f} ms")
                print(f"  Min: {stats.get('min_ms', 0):.2f} ms")
                print(f"  Max: {stats.get('max_ms', 0):.2f} ms")
                print(f"  P95: {stats.get('p95_ms', 0):.2f} ms")
        
        # Disable motor
        try:
            self.client.disable(motor_id)
        except:
            pass
            
        return motor_results
    
    def profile_scaling(self, motor_ids):
        """Profile how latency scales with number of motors."""
        print("\n--- Profiling Multi-Motor Scaling ---")
        
        # Enable all test motors
        active_motors = []
        for motor_id in motor_ids:
            try:
                self.client.enable(motor_id)
                active_motors.append(motor_id)
                time.sleep(0.1)
            except:
                print(f"Failed to enable motor {motor_id}")
        
        if not active_motors:
            print("No motors could be enabled")
            return {}
        
        # Test command for scaling
        def mit_command(m):
            self.client.set_position(m, 0.0, 0.0, 10.0, 1.0, 0.0)
        
        scaling_results = {}
        
        # Test with increasing number of motors
        for num_motors in range(1, len(active_motors) + 1):
            test_motors = active_motors[:num_motors]
            print(f"\nTesting with {num_motors} motor(s): {test_motors}")
            
            stats = self.measure_multi_motor_latency(test_motors, mit_command)
            scaling_results[num_motors] = stats
            
            if stats:
                print(f"  Total time for {num_motors} motors: {stats['avg_ms']:.2f} ms")
                print(f"  Per-motor average: {stats['avg_ms']/num_motors:.2f} ms")
        
        # Disable all motors
        for motor_id in active_motors:
            try:
                self.client.disable(motor_id)
            except:
                pass
        
        return scaling_results
    
    def generate_report(self, output_file="latency_report.json"):
        """Generate a comprehensive latency report."""
        report = {
            'timestamp': datetime.now().isoformat(),
            'configuration': {
                'bitrate': BITRATE,
                'num_samples': NUM_SAMPLES,
                'warmup_samples': WARMUP_SAMPLES
            },
            'results': self.results
        }
        
        with open(output_file, 'w') as f:
            json.dump(report, f, indent=2)
        
        print(f"\nReport saved to {output_file}")
        
        # Print summary
        self._print_summary()
    
    def _print_summary(self):
        """Print a summary of findings."""
        print("\n" + "="*60)
        print("LATENCY MEASUREMENT SUMMARY")
        print("="*60)
        
        # Find best and worst performers
        all_latencies = []
        for test_type, motors in self.results.items():
            if test_type == 'scaling':
                continue
            for motor, commands in motors.items():
                for cmd, stats in commands.items():
                    if 'avg_ms' in stats:
                        all_latencies.append({
                            'test': f"{test_type}/{motor}/{cmd}",
                            'avg_ms': stats['avg_ms'],
                            'p95_ms': stats['p95_ms']
                        })
        
        if all_latencies:
            # Sort by average latency
            sorted_by_avg = sorted(all_latencies, key=lambda x: x['avg_ms'])
            
            print("\nFastest Operations:")
            for item in sorted_by_avg[:3]:
                print(f"  {item['test']}: {item['avg_ms']:.2f} ms avg")
            
            print("\nSlowest Operations:")
            for item in sorted_by_avg[-3:]:
                print(f"  {item['test']}: {item['avg_ms']:.2f} ms avg")
        
        # Scaling summary
        if 'scaling' in self.results:
            print("\nMulti-Motor Scaling:")
            for num_motors, stats in sorted(self.results['scaling'].items()):
                if 'avg_ms' in stats:
                    per_motor = stats['avg_ms'] / int(num_motors)
                    print(f"  {num_motors} motors: {stats['avg_ms']:.2f} ms total "
                          f"({per_motor:.2f} ms per motor)")
        
        print("\nOptimization Suggestions:")
        print("  1. Use batch commands when possible")
        print("  2. Minimize parameter reads during control loops")
        print("  3. Consider parallel command sending for multi-motor control")
        print("  4. Profile your specific use case for targeted optimization")


def main():
    """Main latency profiling routine."""
    print("RS03 Command Latency Measurement Tool")
    print("====================================")
    
    # Initialize CAN
    try:
        bus = can.Bus(interface='socketcan', channel='can0', bitrate=BITRATE)
        client = Client(bus)
    except Exception as e:
        print(f"ERROR: Failed to initialize CAN bus: {e}")
        return 1
    
    profiler = LatencyProfiler(client)
    
    try:
        # Test 1: Single motor command profiling
        print("\n1. SINGLE MOTOR COMMAND PROFILING")
        single_motor_results = {}
        test_motor = TEST_MOTOR_IDS[0] if TEST_MOTOR_IDS else 101
        
        results = profiler.profile_motor_commands(test_motor)
        if results:
            single_motor_results[test_motor] = results
        
        profiler.results['single_motor'] = single_motor_results
        
        # Test 2: Multi-motor scaling
        print("\n2. MULTI-MOTOR SCALING TEST")
        scaling_results = profiler.profile_scaling(TEST_MOTOR_IDS[:3])  # Test up to 3 motors
        profiler.results['scaling'] = scaling_results
        
        # Test 3: Continuous streaming test
        print("\n3. CONTINUOUS STREAMING TEST")
        print("Testing sustained command rate...")
        
        test_motor = TEST_MOTOR_IDS[0] if TEST_MOTOR_IDS else 101
        try:
            client.enable(test_motor)
            
            # Stream commands for 5 seconds
            start_time = time.time()
            command_count = 0
            
            while time.time() - start_time < 5.0:
                client.set_position(test_motor, 0.0, 0.0, 10.0, 1.0, 0.0)
                command_count += 1
                time.sleep(0.001)  # 1kHz target rate
            
            duration = time.time() - start_time
            actual_rate = command_count / duration
            
            print(f"  Commands sent: {command_count}")
            print(f"  Duration: {duration:.2f} seconds")
            print(f"  Actual rate: {actual_rate:.1f} Hz")
            print(f"  Target rate: 1000 Hz")
            
            client.disable(test_motor)
            
            profiler.results['streaming'] = {
                'commands': command_count,
                'duration_s': duration,
                'rate_hz': actual_rate
            }
            
        except Exception as e:
            print(f"  Streaming test failed: {e}")
        
        # Generate report
        profiler.generate_report()
        
    except KeyboardInterrupt:
        print("\n\nProfiling interrupted by user")
    except Exception as e:
        print(f"\n\nERROR: Unexpected error: {e}")
    finally:
        # Cleanup
        print("\nCleaning up...")
        for motor_id in TEST_MOTOR_IDS:
            try:
                client.disable(motor_id)
            except:
                pass
        bus.shutdown()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

