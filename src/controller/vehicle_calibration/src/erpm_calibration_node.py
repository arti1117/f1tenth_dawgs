#!/usr/bin/env python3
"""
ERPM Gain Calibration Node for F1TENTH Vehicle

This node calibrates the relationship between commanded speed and actual ERPM
(Electronic Rotations Per Minute) to ensure accurate speed control.

The calibration process:
1. Commands a series of speed setpoints
2. Measures actual vehicle speed via odometry
3. Records ERPM values from the motor controller
4. Fits a calibration curve to determine the optimal gain

Author: F1TENTH DAWGS
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize
from scipy.signal import savgol_filter
import threading
import time
import math
import json
import os

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String, Int32
from sensor_msgs.msg import JointState
from vesc_msgs.msg import VescStateStamped


class ERPMCalibrationNode(Node):
    def __init__(self):
        super().__init__('erpm_calibration_node')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_speed', 0.5),           # Minimum calibration speed (m/s)
                ('max_speed', 4.0),           # Maximum calibration speed (m/s)
                ('speed_increment', 0.25),    # Speed increment per test (m/s)
                ('test_duration', 8.0),       # Duration for each speed test (s)
                ('settling_time', 2.0),       # Time to let speed settle before recording (s)
                ('data_collection_rate', 50), # Data collection rate (Hz)
                ('wheelbase', 0.33),          # Vehicle wheelbase (m)
                ('wheel_radius', 0.05),       # Wheel radius (m)
                ('gear_ratio', 1.0),          # Gear ratio
                ('plot_results', True),       # Enable result plotting
                ('save_calibration', True),   # Save calibration data
                ('output_directory', '/home/dawgs_nx/f1tenth_dawgs/data/calibration'),
                ('initial_erpm_gain', 4000),  # Initial ERPM gain estimate
                ('motor_poles', 4),           # Number of motor poles
                ('max_erpm', 20000),          # Maximum ERPM limit
                ('tolerance', 0.05)           # Speed tolerance for steady state (m/s)
            ]
        )

        # Get parameters
        self.min_speed = self.get_parameter('min_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        self.speed_increment = self.get_parameter('speed_increment').value
        self.test_duration = self.get_parameter('test_duration').value
        self.settling_time = self.get_parameter('settling_time').value
        self.data_rate = self.get_parameter('data_collection_rate').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.plot_results = self.get_parameter('plot_results').value
        self.save_calibration = self.get_parameter('save_calibration').value
        self.output_dir = self.get_parameter('output_directory').value
        self.initial_erpm_gain = self.get_parameter('initial_erpm_gain').value
        self.motor_poles = self.get_parameter('motor_poles').value
        self.max_erpm = self.get_parameter('max_erpm').value
        self.tolerance = self.get_parameter('tolerance').value

        # QoS Profiles
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publishers
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            reliable_qos
        )

        self.status_pub = self.create_publisher(
            String,
            '/erpm_calibration/status',
            reliable_qos
        )

        self.erpm_gain_pub = self.create_publisher(
            Float32,
            '/erpm_calibration/gain',
            reliable_qos
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            best_effort_qos
        )

        self.erpm_sub = self.create_subscription(
            VescStateStamped,
            '/sensors/core',
            self.erpm_callback,
            best_effort_qos
        )

        # Alternative ERPM source (joint states)
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            best_effort_qos
        )

        self.start_sub = self.create_subscription(
            Bool,
            '/erpm_calibration/start',
            self.start_calibration_callback,
            reliable_qos
        )

        self.stop_sub = self.create_subscription(
            Bool,
            '/erpm_calibration/stop',
            self.stop_calibration_callback,
            reliable_qos
        )

        # Data storage
        self.calibration_data = {
            'timestamp': [],
            'commanded_speed': [],
            'actual_speed': [],
            'erpm': [],
            'speed_error': [],
            'steady_state': []
        }

        # Calibration results
        self.calibration_results = {
            'speed_setpoints': [],
            'average_speeds': [],
            'average_erpms': [],
            'optimal_gain': None,
            'r_squared': 0.0,
            'rmse': 0.0
        }

        # State variables
        self.current_speed = 0.0
        self.commanded_speed = 0.0
        self.current_erpm = 0
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0

        # Calibration control
        self.calibration_active = False
        self.current_test_speed = self.min_speed
        self.test_start_time = None
        self.current_phase = "idle"  # idle, testing, analyzing, completed
        self.speed_stable = False
        self.stable_start_time = None

        # Timer for calibration execution
        self.calibration_timer = self.create_timer(1.0/self.data_rate, self.calibration_callback)

        # Safety
        self.emergency_stop = False

        self.get_logger().info("ERPM Calibration Node initialized")
        self.get_logger().info(f"Speed range: {self.min_speed} - {self.max_speed} m/s")
        self.get_logger().info(f"Initial ERPM gain: {self.initial_erpm_gain}")

    def odom_callback(self, msg):
        """Odometry callback for actual speed measurement"""
        self.current_speed = math.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )
        self.vehicle_x = msg.pose.pose.position.x
        self.vehicle_y = msg.pose.pose.position.y

    def erpm_callback(self, msg):
        """ERPM callback from motor controller"""
        self.current_erpm = msg.state.speed

    def joint_states_callback(self, msg):
        """Alternative ERPM source from joint states (if available)"""
        try:
            if 'motor_speed' in msg.name:
                idx = msg.name.index('motor_speed')
                # Convert angular velocity to ERPM
                motor_rpm = (msg.velocity[idx] * 60) / (2 * math.pi)
                self.current_erpm = int(motor_rpm * self.motor_poles / 2)
        except (ValueError, IndexError):
            pass

    def start_calibration_callback(self, msg):
        """Start ERPM calibration"""
        if msg.data and not self.calibration_active:
            self.start_erpm_calibration()

    def stop_calibration_callback(self, msg):
        """Stop ERPM calibration"""
        if msg.data:
            self.stop_erpm_calibration()

    def start_erpm_calibration(self):
        """Initialize and start ERPM calibration"""
        self.get_logger().info("Starting ERPM Calibration")

        # Reset calibration state
        self.calibration_active = True
        self.current_test_speed = self.min_speed
        self.test_start_time = time.time()
        self.current_phase = "testing"
        self.emergency_stop = False
        self.speed_stable = False
        self.stable_start_time = None

        # Clear previous data
        for key in self.calibration_data.keys():
            self.calibration_data[key].clear()

        for key in self.calibration_results.keys():
            if isinstance(self.calibration_results[key], list):
                self.calibration_results[key].clear()

        # Publish status
        self.publish_status(f"Calibration started - Testing speed: {self.current_test_speed:.1f} m/s")

    def stop_erpm_calibration(self):
        """Stop ERPM calibration"""
        self.get_logger().info("Stopping ERPM Calibration")

        self.calibration_active = False
        self.current_phase = "completed"
        self.emergency_stop = False

        # Send stop command
        self.send_drive_command(0.0)

        # Analyze calibration data if sufficient data available
        if len(self.calibration_results['speed_setpoints']) >= 3:
            self.analyze_calibration_data()
            if self.save_calibration:
                self.save_calibration_data()
            if self.plot_results:
                self.plot_calibration_results()
        else:
            self.get_logger().warn("Insufficient data for calibration analysis")

        self.publish_status("Calibration completed")

    def calibration_callback(self):
        """Main calibration execution loop"""
        if not self.calibration_active or self.emergency_stop:
            return

        current_time = time.time()

        # Check if current speed test is complete
        elapsed_time = current_time - self.test_start_time

        # Check if speed is stable (within tolerance)
        speed_error = abs(self.current_speed - self.commanded_speed)
        if speed_error < self.tolerance:
            if not self.speed_stable:
                self.speed_stable = True
                self.stable_start_time = current_time
                self.get_logger().info(f"Speed stabilized at {self.current_speed:.2f} m/s")
        else:
            self.speed_stable = False
            self.stable_start_time = None

        # Collect data if in stable region
        is_steady_state = (
            self.speed_stable and
            self.stable_start_time and
            (current_time - self.stable_start_time) >= self.settling_time
        )

        # Record data
        self.collect_calibration_data(current_time, is_steady_state)

        # Check if test duration is complete
        if elapsed_time >= self.test_duration:
            # Process current speed data
            self.process_current_speed_test()

            # Move to next speed
            self.current_test_speed += self.speed_increment

            if self.current_test_speed > self.max_speed:
                # Calibration complete
                self.stop_erpm_calibration()
                return

            # Start next speed test
            self.test_start_time = current_time
            self.speed_stable = False
            self.stable_start_time = None

            self.get_logger().info(f"Testing speed: {self.current_test_speed:.1f} m/s")
            self.publish_status(f"Testing speed: {self.current_test_speed:.1f} m/s")

        # Command current test speed
        self.commanded_speed = self.current_test_speed
        self.send_drive_command(self.current_test_speed)

    def send_drive_command(self, speed):
        """Send drive command with zero steering"""
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = 0.0

        self.drive_pub.publish(drive_msg)

    def collect_calibration_data(self, timestamp, is_steady_state):
        """Collect calibration data point"""
        speed_error = self.current_speed - self.commanded_speed

        self.calibration_data['timestamp'].append(timestamp)
        self.calibration_data['commanded_speed'].append(self.commanded_speed)
        self.calibration_data['actual_speed'].append(self.current_speed)
        self.calibration_data['erpm'].append(self.current_erpm)
        self.calibration_data['speed_error'].append(speed_error)
        self.calibration_data['steady_state'].append(is_steady_state)

    def process_current_speed_test(self):
        """Process data from current speed test"""
        if not self.calibration_data['timestamp']:
            return

        # Filter steady-state data
        steady_indices = [i for i, steady in enumerate(self.calibration_data['steady_state']) if steady]

        if len(steady_indices) < 10:  # Minimum data points for reliable average
            self.get_logger().warn(f"Insufficient steady-state data for speed {self.current_test_speed:.1f} m/s")
            return

        # Calculate averages for steady-state data
        steady_speeds = [self.calibration_data['actual_speed'][i] for i in steady_indices]
        steady_erpms = [self.calibration_data['erpm'][i] for i in steady_indices]

        avg_speed = np.mean(steady_speeds)
        avg_erpm = np.mean(steady_erpms)

        self.calibration_results['speed_setpoints'].append(self.current_test_speed)
        self.calibration_results['average_speeds'].append(avg_speed)
        self.calibration_results['average_erpms'].append(avg_erpm)

        self.get_logger().info(
            f"Speed {self.current_test_speed:.1f} m/s -> "
            f"Actual: {avg_speed:.2f} m/s, ERPM: {avg_erpm:.0f}"
        )

    def analyze_calibration_data(self):
        """Analyze calibration data and determine optimal ERPM gain"""
        if len(self.calibration_results['speed_setpoints']) < 3:
            self.get_logger().error("Insufficient data for calibration analysis")
            return

        speeds = np.array(self.calibration_results['average_speeds'])
        erpms = np.array(self.calibration_results['average_erpms'])

        # Remove zero speed data points
        non_zero_mask = speeds > 0.1
        speeds_nz = speeds[non_zero_mask]
        erpms_nz = erpms[non_zero_mask]

        if len(speeds_nz) < 2:
            self.get_logger().error("Insufficient non-zero speed data for calibration")
            return

        # Fit linear relationship: ERPM = gain * speed
        # Using least squares fitting
        try:
            # Method 1: Linear regression through origin (ERPM = gain * speed)
            gain_through_origin = np.sum(speeds_nz * erpms_nz) / np.sum(speeds_nz**2)

            # Method 2: Linear regression with intercept (ERPM = gain * speed + offset)
            coeffs = np.polyfit(speeds_nz, erpms_nz, 1)
            gain_with_intercept = coeffs[0]
            intercept = coeffs[1]

            # Choose method based on intercept magnitude
            if abs(intercept) < 500:  # Small intercept, use through-origin model
                optimal_gain = gain_through_origin
                predicted_erpms = optimal_gain * speeds_nz
            else:
                optimal_gain = gain_with_intercept
                predicted_erpms = optimal_gain * speeds_nz + intercept

            # Calculate goodness of fit
            ss_res = np.sum((erpms_nz - predicted_erpms)**2)
            ss_tot = np.sum((erpms_nz - np.mean(erpms_nz))**2)
            r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

            # Calculate RMSE
            rmse = np.sqrt(np.mean((erpms_nz - predicted_erpms)**2))

            # Store results
            self.calibration_results['optimal_gain'] = optimal_gain
            self.calibration_results['r_squared'] = r_squared
            self.calibration_results['rmse'] = rmse

            # Publish optimal gain
            gain_msg = Float32()
            gain_msg.data = float(optimal_gain)
            self.erpm_gain_pub.publish(gain_msg)

            self.get_logger().info("=== ERPM Calibration Results ===")
            self.get_logger().info(f"Optimal ERPM Gain: {optimal_gain:.0f}")
            self.get_logger().info(f"R-squared: {r_squared:.4f}")
            self.get_logger().info(f"RMSE: {rmse:.2f} ERPM")

            if abs(intercept) >= 500:
                self.get_logger().info(f"Intercept: {intercept:.0f} ERPM")

            # Quality assessment
            if r_squared > 0.95:
                self.get_logger().info("Calibration quality: Excellent")
            elif r_squared > 0.90:
                self.get_logger().info("Calibration quality: Good")
            elif r_squared > 0.80:
                self.get_logger().info("Calibration quality: Fair")
            else:
                self.get_logger().warn("Calibration quality: Poor - Consider recalibrating")

        except Exception as e:
            self.get_logger().error(f"Failed to analyze calibration data: {e}")

    def plot_calibration_results(self):
        """Plot calibration results"""
        try:
            plt.figure(figsize=(15, 10))

            # Plot 1: Speed vs ERPM relationship
            plt.subplot(2, 2, 1)
            speeds = np.array(self.calibration_results['average_speeds'])
            erpms = np.array(self.calibration_results['average_erpms'])

            plt.scatter(speeds, erpms, c='blue', s=50, alpha=0.7, label='Measured Data')

            # Plot fitted line
            if self.calibration_results['optimal_gain']:
                speed_range = np.linspace(0, max(speeds) * 1.1, 100)
                predicted_erpms = self.calibration_results['optimal_gain'] * speed_range
                plt.plot(speed_range, predicted_erpms, 'r-', linewidth=2,
                        label=f'Fitted Line (Gain: {self.calibration_results["optimal_gain"]:.0f})')

            plt.xlabel('Speed (m/s)')
            plt.ylabel('ERPM')
            plt.title('Speed vs ERPM Calibration')
            plt.legend()
            plt.grid(True, alpha=0.3)

            # Plot 2: Speed tracking performance
            plt.subplot(2, 2, 2)
            setpoints = self.calibration_results['speed_setpoints']
            actual_speeds = self.calibration_results['average_speeds']

            plt.scatter(setpoints, actual_speeds, c='green', s=50, alpha=0.7, label='Actual Speed')
            plt.plot([0, max(setpoints)], [0, max(setpoints)], 'k--', alpha=0.5, label='Perfect Tracking')

            plt.xlabel('Commanded Speed (m/s)')
            plt.ylabel('Actual Speed (m/s)')
            plt.title('Speed Tracking Performance')
            plt.legend()
            plt.grid(True, alpha=0.3)

            # Plot 3: Time series data (last few speed tests)
            plt.subplot(2, 2, 3)
            if len(self.calibration_data['timestamp']) > 100:
                times = np.array(self.calibration_data['timestamp'])
                times = (times - times[0])  # Relative time

                plt.plot(times, self.calibration_data['commanded_speed'], 'r-',
                        linewidth=2, label='Commanded Speed', alpha=0.8)
                plt.plot(times, self.calibration_data['actual_speed'], 'b-',
                        linewidth=1, label='Actual Speed')

                plt.xlabel('Time (s)')
                plt.ylabel('Speed (m/s)')
                plt.title('Speed Tracking Over Time')
                plt.legend()
                plt.grid(True, alpha=0.3)

            # Plot 4: ERPM vs Time
            plt.subplot(2, 2, 4)
            if len(self.calibration_data['timestamp']) > 100:
                plt.plot(times, self.calibration_data['erpm'], 'purple', linewidth=1)
                plt.xlabel('Time (s)')
                plt.ylabel('ERPM')
                plt.title('ERPM Over Time')
                plt.grid(True, alpha=0.3)

            plt.tight_layout()

            # Save plot if requested
            if self.save_calibration:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                plot_filename = os.path.join(self.output_dir, f"erpm_calibration_{timestamp}.png")
                plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
                self.get_logger().info(f"Calibration plot saved to: {plot_filename}")

            plt.show(block=False)

        except Exception as e:
            self.get_logger().error(f"Failed to plot calibration results: {e}")

    def save_calibration_data(self):
        """Save calibration data and results"""
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")

            # Save raw data
            import pandas as pd
            df = pd.DataFrame(self.calibration_data)
            data_filename = os.path.join(self.output_dir, f"erpm_calibration_data_{timestamp}.csv")
            df.to_csv(data_filename, index=False)

            # Save calibration results
            results_filename = os.path.join(self.output_dir, f"erpm_calibration_results_{timestamp}.json")
            with open(results_filename, 'w') as f:
                # Convert numpy types to Python native types for JSON serialization
                json_results = {}
                for key, value in self.calibration_results.items():
                    if isinstance(value, np.ndarray):
                        json_results[key] = value.tolist()
                    elif isinstance(value, (np.integer, np.floating)):
                        json_results[key] = float(value)
                    else:
                        json_results[key] = value

                json.dump(json_results, f, indent=2)

            # Save optimal gain to a simple config file
            if self.calibration_results['optimal_gain']:
                config_filename = os.path.join(self.output_dir, "erpm_gain_config.txt")
                with open(config_filename, 'w') as f:
                    f.write(f"# ERPM Gain Calibration Results\n")
                    f.write(f"# Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                    f.write(f"erpm_gain: {self.calibration_results['optimal_gain']:.0f}\n")
                    f.write(f"r_squared: {self.calibration_results['r_squared']:.4f}\n")
                    f.write(f"rmse: {self.calibration_results['rmse']:.2f}\n")

            self.get_logger().info(f"Calibration data saved to: {self.output_dir}")

        except Exception as e:
            self.get_logger().error(f"Failed to save calibration data: {e}")

    def publish_status(self, message):
        """Publish status message"""
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    node = ERPMCalibrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("ERPM Calibration Node interrupted by user")
        if node.calibration_active:
            node.stop_erpm_calibration()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()