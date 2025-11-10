#!/usr/bin/env python3
"""
Trajectory following examples for RS03 motors.
Demonstrates sine wave, trapezoidal, and custom trajectory profiles.
"""
import os
import sys
import time
import can
from robstride import Client, RunMode
from safety_utils import MotorSafety, RateLimiter
import numpy as np
import math
from dataclasses import dataclass
from typing import List, Tuple, Callable
from collections import defaultdict

# Configuration
PORT = os.getenv('PORT', '/dev/ttyACM0')
BITRATE = int(os.getenv('BITRATE', 1000000))

# Trajectory parameters
CONTROL_FREQ = 100  # Hz
DT = 1.0 / CONTROL_FREQ


@dataclass
class TrajectoryPoint:
    """Single point in a trajectory."""
    time: float
    position: float
    velocity: float
    acceleration: float
    
    
class TrajectoryGenerator:
    """Generates various trajectory profiles."""
    
    @staticmethod
    def sine_wave(amplitude: float, frequency: float, duration: float) -> List[TrajectoryPoint]:
        """Generate sine wave trajectory."""
        points = []
        t = 0
        omega = 2 * math.pi * frequency
        
        while t <= duration:
            pos = amplitude * math.sin(omega * t)
            vel = amplitude * omega * math.cos(omega * t)
            acc = -amplitude * omega**2 * math.sin(omega * t)
            
            points.append(TrajectoryPoint(t, pos, vel, acc))
            t += DT
            
        return points
    
    @staticmethod
    def trapezoidal_profile(distance: float, max_vel: float, max_acc: float) -> List[TrajectoryPoint]:
        """Generate trapezoidal velocity profile for point-to-point motion."""
        points = []
        
        # Calculate profile parameters
        accel_time = max_vel / max_acc
        accel_dist = 0.5 * max_acc * accel_time**2
        
        # Check if we reach max velocity
        if 2 * accel_dist > abs(distance):
            # Triangular profile (doesn't reach max velocity)
            accel_time = math.sqrt(abs(distance) / max_acc)
            accel_dist = 0.5 * max_acc * accel_time**2
            cruise_time = 0
            cruise_dist = 0
            peak_vel = max_acc * accel_time
        else:
            # Full trapezoidal profile
            cruise_dist = abs(distance) - 2 * accel_dist
            cruise_time = cruise_dist / max_vel
            peak_vel = max_vel
        
        total_time = 2 * accel_time + cruise_time
        sign = 1 if distance > 0 else -1
        
        t = 0
        while t <= total_time + DT:
            if t <= accel_time:
                # Acceleration phase
                pos = sign * 0.5 * max_acc * t**2
                vel = sign * max_acc * t
                acc = sign * max_acc
            elif t <= accel_time + cruise_time:
                # Cruise phase
                t_cruise = t - accel_time
                pos = sign * (accel_dist + peak_vel * t_cruise)
                vel = sign * peak_vel
                acc = 0
            else:
                # Deceleration phase
                t_decel = t - accel_time - cruise_time
                t_remaining = accel_time - t_decel
                pos = sign * (distance - 0.5 * max_acc * t_remaining**2)
                vel = sign * max_acc * t_remaining
                acc = -sign * max_acc
                
                if t_remaining <= 0:
                    pos = sign * distance
                    vel = 0
                    acc = 0
            
            points.append(TrajectoryPoint(t, pos, vel, acc))
            t += DT
            
        return points
    
    @staticmethod
    def circular_path(radius: float, angular_freq: float, duration: float) -> List[Tuple[TrajectoryPoint, TrajectoryPoint]]:
        """Generate circular path for 2 motors (X, Y coordinates)."""
        points = []
        t = 0
        
        while t <= duration:
            # X motor (cosine)
            x_pos = radius * math.cos(angular_freq * t)
            x_vel = -radius * angular_freq * math.sin(angular_freq * t)
            x_acc = -radius * angular_freq**2 * math.cos(angular_freq * t)
            
            # Y motor (sine)
            y_pos = radius * math.sin(angular_freq * t)
            y_vel = radius * angular_freq * math.cos(angular_freq * t)
            y_acc = -radius * angular_freq**2 * math.sin(angular_freq * t)
            
            x_point = TrajectoryPoint(t, x_pos, x_vel, x_acc)
            y_point = TrajectoryPoint(t, y_pos, y_vel, y_acc)
            
            points.append((x_point, y_point))
            t += DT
            
        return points
    
    @staticmethod
    def multi_motor_wave(num_motors: int, amplitude: float, frequency: float, 
                        phase_shift: float, duration: float) -> List[List[TrajectoryPoint]]:
        """Generate phase-shifted sine waves for multiple motors."""
        all_trajectories = []
        
        for motor_idx in range(num_motors):
            phase = motor_idx * phase_shift
            points = []
            t = 0
            omega = 2 * math.pi * frequency
            
            while t <= duration:
                pos = amplitude * math.sin(omega * t + phase)
                vel = amplitude * omega * math.cos(omega * t + phase)
                acc = -amplitude * omega**2 * math.sin(omega * t + phase)
                
                points.append(TrajectoryPoint(t, pos, vel, acc))
                t += DT
                
            all_trajectories.append(points)
            
        return all_trajectories


class TrajectoryTracker:
    """Tracks trajectory following performance."""
    
    def __init__(self):
        self.position_errors = defaultdict(list)
        self.velocity_errors = defaultdict(list)
        self.actual_positions = defaultdict(list)
        self.actual_velocities = defaultdict(list)
        self.timestamps = []
        
    def record(self, timestamp: float, motor_id: int, 
               desired_pos: float, desired_vel: float,
               actual_pos: float, actual_vel: float = None):
        """Record tracking data."""
        self.timestamps.append(timestamp)
        self.position_errors[motor_id].append(desired_pos - actual_pos)
        self.actual_positions[motor_id].append(actual_pos)
        
        if actual_vel is not None:
            self.velocity_errors[motor_id].append(desired_vel - actual_vel)
            self.actual_velocities[motor_id].append(actual_vel)
    
    def get_statistics(self, motor_id: int) -> dict:
        """Calculate tracking statistics for a motor."""
        pos_errors = np.array(self.position_errors[motor_id])
        
        stats = {
            'position_rmse': np.sqrt(np.mean(pos_errors**2)),
            'position_max_error': np.max(np.abs(pos_errors)),
            'position_mean_error': np.mean(pos_errors),
            'position_std_error': np.std(pos_errors)
        }
        
        if self.velocity_errors[motor_id]:
            vel_errors = np.array(self.velocity_errors[motor_id])
            stats.update({
                'velocity_rmse': np.sqrt(np.mean(vel_errors**2)),
                'velocity_max_error': np.max(np.abs(vel_errors)),
            })
            
        return stats


class TrajectoryFollower:
    """Executes trajectory following with various control strategies."""
    
    def __init__(self, client, motor_ids):
        self.client = client
        self.motor_ids = motor_ids
        self.safety = MotorSafety(client, motor_ids)
        self.rate_limiter = RateLimiter(min_interval=DT)
        self.tracker = TrajectoryTracker()
        
    def follow_trajectory(self, motor_id: int, trajectory: List[TrajectoryPoint], 
                         kp: float = 50.0, kd: float = 5.0, kff_vel: float = 1.0):
        """Follow a single motor trajectory."""
        print(f"\nFollowing trajectory on motor {motor_id}")
        print(f"  Duration: {trajectory[-1].time:.1f}s")
        print(f"  Control: Kp={kp}, Kd={kd}, Kff_vel={kff_vel}")
        
        # Enable motor
        if not self.safety.safe_enable(motor_id):
            print(f"Failed to enable motor {motor_id}")
            return False
            
        # Get initial position
        try:
            initial_pos = self.client.read_param(motor_id, 'mechpos')
        except:
            initial_pos = 0
            
        # Offset trajectory to start from current position
        offset = initial_pos - trajectory[0].position
        
        start_time = time.time()
        point_idx = 0
        
        try:
            while point_idx < len(trajectory):
                current_time = time.time() - start_time
                
                # Find the right trajectory point
                while (point_idx < len(trajectory) - 1 and 
                       trajectory[point_idx + 1].time <= current_time):
                    point_idx += 1
                
                point = trajectory[point_idx]
                
                # Apply offset and feedforward
                target_pos = point.position + offset
                ff_velocity = point.velocity * kff_vel
                
                # Send command
                self.client.set_position(motor_id, target_pos, ff_velocity, kp, kd, 0.0)
                
                # Record tracking data
                try:
                    actual_pos = self.client.read_param(motor_id, 'mechpos')
                    self.tracker.record(current_time, motor_id, 
                                      target_pos, point.velocity,
                                      actual_pos)
                except:
                    pass
                
                self.rate_limiter.wait()
                
                # Check if trajectory is complete
                if current_time > trajectory[-1].time:
                    break
                    
        except KeyboardInterrupt:
            print("\nTrajectory interrupted")
            return False
        finally:
            # Hold final position briefly
            final_pos = trajectory[-1].position + offset
            for _ in range(int(CONTROL_FREQ)):  # 1 second
                self.client.set_position(motor_id, final_pos, 0.0, kp, kd, 0.0)
                self.rate_limiter.wait()
                
            self.safety.safe_disable(motor_id)
            
        # Print statistics
        stats = self.tracker.get_statistics(motor_id)
        print(f"\nTracking performance:")
        print(f"  Position RMSE: {stats['position_rmse']:.4f} rad")
        print(f"  Max position error: {stats['position_max_error']:.4f} rad")
        
        return True
    
    def follow_multi_trajectory(self, trajectories: dict, kp: float = 50.0, kd: float = 5.0):
        """Follow trajectories on multiple motors simultaneously."""
        motor_ids = list(trajectories.keys())
        print(f"\nFollowing trajectories on {len(motor_ids)} motors")
        
        # Enable all motors
        for motor_id in motor_ids:
            if not self.safety.safe_enable(motor_id):
                print(f"Failed to enable motor {motor_id}")
                return False
        
        # Get initial positions and calculate offsets
        offsets = {}
        for motor_id in motor_ids:
            try:
                initial_pos = self.client.read_param(motor_id, 'mechpos')
                offsets[motor_id] = initial_pos - trajectories[motor_id][0].position
            except:
                offsets[motor_id] = 0
        
        # Find longest trajectory
        max_duration = max(traj[-1].time for traj in trajectories.values())
        
        start_time = time.time()
        point_indices = {m: 0 for m in motor_ids}
        
        try:
            while True:
                current_time = time.time() - start_time
                
                if current_time > max_duration:
                    break
                
                # Command each motor
                for motor_id in motor_ids:
                    traj = trajectories[motor_id]
                    idx = point_indices[motor_id]
                    
                    # Find current point
                    while (idx < len(traj) - 1 and traj[idx + 1].time <= current_time):
                        idx += 1
                    point_indices[motor_id] = idx
                    
                    if idx < len(traj):
                        point = traj[idx]
                        target_pos = point.position + offsets[motor_id]
                        
                        self.client.set_position(motor_id, target_pos, point.velocity, 
                                               kp, kd, 0.0)
                        
                        # Record tracking (only occasionally to reduce overhead)
                        if int(current_time * 10) % 10 == 0:  # Every 0.1s
                            try:
                                actual_pos = self.client.read_param(motor_id, 'mechpos')
                                self.tracker.record(current_time, motor_id,
                                                  target_pos, point.velocity,
                                                  actual_pos)
                            except:
                                pass
                
                self.rate_limiter.wait()
                
        except KeyboardInterrupt:
            print("\nTrajectories interrupted")
        finally:
            # Disable all motors
            for motor_id in motor_ids:
                self.safety.safe_disable(motor_id)
        
        # Print statistics
        print("\nTracking performance:")
        for motor_id in motor_ids:
            stats = self.tracker.get_statistics(motor_id)
            print(f"  Motor {motor_id} - Position RMSE: {stats['position_rmse']:.4f} rad")


def run_trajectory_demos(client):
    """Run various trajectory following demonstrations."""
    generator = TrajectoryGenerator()
    
    # Demo 1: Single motor sine wave
    print("\n" + "="*60)
    print("DEMO 1: Sine Wave Trajectory")
    print("="*60)
    
    motor_id = 101
    sine_traj = generator.sine_wave(amplitude=0.5, frequency=0.5, duration=10.0)
    
    follower = TrajectoryFollower(client, [motor_id])
    follower.follow_trajectory(motor_id, sine_traj, kp=40.0, kd=4.0)
    
    # Demo 2: Trapezoidal profile
    print("\n" + "="*60)
    print("DEMO 2: Trapezoidal Velocity Profile")
    print("="*60)
    
    trap_traj = generator.trapezoidal_profile(distance=2.0, max_vel=1.0, max_acc=2.0)
    
    follower = TrajectoryFollower(client, [motor_id])
    follower.follow_trajectory(motor_id, trap_traj, kp=50.0, kd=5.0)
    
    # Demo 3: Multi-motor wave
    print("\n" + "="*60)
    print("DEMO 3: Multi-Motor Phase-Shifted Wave")
    print("="*60)
    
    motor_ids = [101, 102, 103]
    wave_trajs = generator.multi_motor_wave(
        num_motors=len(motor_ids),
        amplitude=0.3,
        frequency=0.3,
        phase_shift=2*math.pi/len(motor_ids),
        duration=10.0
    )
    
    # Convert to dict
    traj_dict = {motor_ids[i]: wave_trajs[i] for i in range(len(motor_ids))}
    
    follower = TrajectoryFollower(client, motor_ids)
    follower.follow_multi_trajectory(traj_dict, kp=35.0, kd=3.5)
    
    # Demo 4: Circular path with 2 motors
    if len(motor_ids) >= 2:
        print("\n" + "="*60)
        print("DEMO 4: Circular Path (2 Motors)")
        print("="*60)
        
        circular_points = generator.circular_path(radius=0.3, angular_freq=0.5, duration=10.0)
        
        # Separate into X and Y trajectories
        x_traj = [p[0] for p in circular_points]
        y_traj = [p[1] for p in circular_points]
        
        traj_dict = {
            motor_ids[0]: x_traj,  # X motor
            motor_ids[1]: y_traj   # Y motor
        }
        
        follower = TrajectoryFollower(client, motor_ids[:2])
        follower.follow_multi_trajectory(traj_dict, kp=40.0, kd=4.0)


def main():
    """Main trajectory following demonstration."""
    print("RS03 Trajectory Following Examples")
    print("==================================")
    
    # Initialize CAN
    try:
        bus = can.Bus(interface='socketcan', channel='can0', bitrate=BITRATE)
        client = Client(bus)
    except Exception as e:
        print(f"ERROR: Failed to initialize CAN bus: {e}")
        return 1
    
    try:
        # Run demonstrations
        run_trajectory_demos(client)
        
        print("\n✓ All trajectory demonstrations complete!")
        print("\nKey takeaways:")
        print("  1. Sine waves test smooth continuous motion")
        print("  2. Trapezoidal profiles optimize point-to-point moves")
        print("  3. Multi-motor coordination enables complex motions")
        print("  4. Tracking performance depends on control gains")
        print("  5. Feedforward improves trajectory following")
        
    except KeyboardInterrupt:
        print("\n\nDemonstration interrupted by user")
    except Exception as e:
        print(f"\n\nERROR: Unexpected error: {e}")
    finally:
        # Cleanup
        bus.shutdown()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

