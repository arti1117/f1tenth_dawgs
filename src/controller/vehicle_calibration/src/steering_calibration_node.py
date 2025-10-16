#!/usr/bin/env python3
"""
Steering Gain Calibration Node for F1TENTH Vehicle

This node calibrates the relationship between commanded steering angle and actual
wheel angle to ensure accurate steering control.

The calibration process:
1. Commands a series of steering angles
2. Measures actual turning radius via odometry
3. Compares with theoretical turning radius from bicycle model
4. Determines optimal steering gain and offset

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
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import JointState
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_pose


class SteeringCalibrationNode(Node):
    def __init__(self):
        super().__init__('steering_calibration_node')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_steering_angle', -0.4),    # Minimum steering angle (rad)
                ('max_steering_angle', 0.4),     # Maximum steering angle (rad)
                ('steering_increment', 0.05),    # Steering increment per test (rad)
                ('test_speed', 1.0),             # Constant test speed (m/s)
                ('test_duration', 12.0),         # Duration for each steering test (s)
                ('settling_time', 2.0),          # Time to let steering settle (s)
                ('data_collection_rate', 50),    # Data collection rate (Hz)
                ('wheelbase', 0.33),             # Vehicle wheelbase (m)
                ('min_turn_radius', 0.5),        # Minimum valid turning radius (m)
                ('max_turn_radius', 20.0),       # Maximum valid turning radius (m)
                ('plot_results', True),          # Enable result plotting
                ('save_calibration', True),      # Save calibration data
                ('output_directory', '/home/dawgs_nx/f1tenth_dawgs/data/calibration'),
                ('initial_steering_gain', 1.0),  # Initial steering gain estimate
                ('initial_steering_offset', 0.0), # Initial steering offset (rad)
                ('position_tolerance', 0.1),     # Position tolerance for circle fitting (m)
                ('min_arc_angle', 1.0),          # Minimum arc angle for valid measurement (rad)
                ('use_least_squares_circle', True) # Use least squares circle fitting
            ]
        )

        # Get parameters
        self.min_steering = self.get_parameter('min_steering_angle').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.steering_increment = self.get_parameter('steering_increment').value
        self.test_speed = self.get_parameter('test_speed').value
        self.test_duration = self.get_parameter('test_duration').value
        self.settling_time = self.get_parameter('settling_time').value
        self.data_rate = self.get_parameter('data_collection_rate').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.min_turn_radius = self.get_parameter('min_turn_radius').value
        self.max_turn_radius = self.get_parameter('max_turn_radius').value
        self.plot_results = self.get_parameter('plot_results').value
        self.save_calibration = self.get_parameter('save_calibration').value
        self.output_dir = self.get_parameter('output_directory').value
        self.initial_steering_gain = self.get_parameter('initial_steering_gain').value
        self.initial_steering_offset = self.get_parameter('initial_steering_offset').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.min_arc_angle = self.get_parameter('min_arc_angle').value
        self.use_least_squares = self.get_parameter('use_least_squares_circle').value

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
            '/steering_calibration/status',
            reliable_qos
        )

        self.steering_gain_pub = self.create_publisher(
            Float32,
            '/steering_calibration/gain',
            reliable_qos
        )

        self.steering_offset_pub = self.create_publisher(
            Float32,
            '/steering_calibration/offset',
            reliable_qos
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            best_effort_qos
        )

        self.start_sub = self.create_subscription(
            Bool,
            '/steering_calibration/start',
            self.start_calibration_callback,
            reliable_qos
        )

        self.stop_sub = self.create_subscription(
            Bool,
            '/steering_calibration/stop',
            self.stop_calibration_callback,
            reliable_qos
        )

        # TF2 Buffer and Listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Data storage
        self.calibration_data = {
            'timestamp': [],
            'commanded_steering': [],
            'position_x': [],
            'position_y': [],
            'heading': [],
            'speed': [],
            'arc_complete': []
        }

        # Calibration results
        self.calibration_results = {
            'steering_commands': [],
            'measured_radii': [],
            'theoretical_radii': [],
            'radius_errors': [],
            'optimal_gain': None,
            'optimal_offset': None,
            'r_squared': 0.0,
            'rmse': 0.0
        }

        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_heading = 0.0
        self.current_speed = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_heading = 0.0

        # Calibration control
        self.calibration_active = False
        self.current_steering_angle = self.min_steering
        self.test_start_time = None
        self.arc_start_time = None
        self.current_phase = "idle"  # idle, testing, analyzing, completed
        self.arc_positions = []

        # Timer for calibration execution
        self.calibration_timer = self.create_timer(1.0/self.data_rate, self.calibration_callback)

        # Safety
        self.emergency_stop = False

        self.get_logger().info("Steering Calibration Node initialized")
        self.get_logger().info(f"Steering range: {math.degrees(self.min_steering):.1f}° - {math.degrees(self.max_steering):.1f}°")
        self.get_logger().info(f"Test speed: {self.test_speed} m/s")

    def odom_callback(self, msg):
        """Odometry callback for position and orientation"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_speed = math.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )

        # Extract heading from quaternion
        quat = msg.pose.pose.orientation
        self.current_heading = math.atan2(
            2 * (quat.w * quat.z + quat.x * quat.y),
            1 - 2 * (quat.y**2 + quat.z**2)
        )

    def start_calibration_callback(self, msg):
        """Start steering calibration"""
        if msg.data and not self.calibration_active:
            self.start_steering_calibration()

    def stop_calibration_callback(self, msg):
        """Stop steering calibration"""
        if msg.data:
            self.stop_steering_calibration()

    def start_steering_calibration(self):
        """Initialize and start steering calibration"""
        self.get_logger().info("Starting Steering Calibration")

        # Reset calibration state
        self.calibration_active = True
        self.current_steering_angle = self.min_steering
        self.test_start_time = time.time()
        self.arc_start_time = None
        self.current_phase = "testing"
        self.emergency_stop = False

        # Record starting position
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_heading = self.current_heading

        # Clear previous data
        for key in self.calibration_data.keys():
            self.calibration_data[key].clear()

        for key in self.calibration_results.keys():
            if isinstance(self.calibration_results[key], list):
                self.calibration_results[key].clear()

        self.arc_positions = []

        # Publish status
        self.publish_status(f"Calibration started - Testing steering: {math.degrees(self.current_steering_angle):.1f}°")

    def stop_steering_calibration(self):
        """Stop steering calibration"""
        self.get_logger().info("Stopping Steering Calibration")

        self.calibration_active = False
        self.current_phase = "completed"
        self.emergency_stop = False

        # Send stop command
        self.send_drive_command(0.0, 0.0)

        # Analyze calibration data if sufficient data available
        if len(self.calibration_results['steering_commands']) >= 3:
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

        # Check if current steering test is complete
        elapsed_time = current_time - self.test_start_time

        # Start recording arc after settling time
        if elapsed_time >= self.settling_time and not self.arc_start_time:
            self.arc_start_time = current_time
            self.arc_positions = []
            self.get_logger().info(f"Recording arc for steering {math.degrees(self.current_steering_angle):.1f}°")

        # Record position data if in arc recording phase
        if self.arc_start_time:
            arc_elapsed = current_time - self.arc_start_time
            arc_complete = arc_elapsed >= (self.test_duration - self.settling_time)

            # Store position data
            self.arc_positions.append([self.current_x, self.current_y, current_time])

            # Collect data
            self.collect_calibration_data(current_time, arc_complete)

        # Check if test duration is complete
        if elapsed_time >= self.test_duration:
            # Process current steering angle test
            self.process_current_steering_test()

            # Move to next steering angle
            self.current_steering_angle += self.steering_increment

            if self.current_steering_angle > self.max_steering:
                # Test positive steering angles if we started with negative
                if self.min_steering < 0 and self.current_steering_angle > 0:
                    # Skip zero and small positive angles if we already tested negative
                    next_angle = self.steering_increment
                    while next_angle <= abs(self.min_steering) and next_angle <= self.max_steering:
                        self.current_steering_angle = next_angle
                        next_angle += self.steering_increment
                        break

                if self.current_steering_angle > self.max_steering:
                    # Calibration complete
                    self.stop_steering_calibration()
                    return

            # Start next steering test
            self.test_start_time = current_time
            self.arc_start_time = None
            self.arc_positions = []

            self.get_logger().info(f"Testing steering: {math.degrees(self.current_steering_angle):.1f}°")
            self.publish_status(f"Testing steering: {math.degrees(self.current_steering_angle):.1f}°")

        # Command current test steering
        self.send_drive_command(self.test_speed, self.current_steering_angle)

    def send_drive_command(self, speed, steering_angle):
        """Send Ackermann drive command"""
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(drive_msg)

    def collect_calibration_data(self, timestamp, arc_complete):
        """Collect calibration data point"""
        self.calibration_data['timestamp'].append(timestamp)
        self.calibration_data['commanded_steering'].append(self.current_steering_angle)
        self.calibration_data['position_x'].append(self.current_x)
        self.calibration_data['position_y'].append(self.current_y)
        self.calibration_data['heading'].append(self.current_heading)
        self.calibration_data['speed'].append(self.current_speed)
        self.calibration_data['arc_complete'].append(arc_complete)

    def process_current_steering_test(self):
        """Process data from current steering test to extract turning radius"""
        if len(self.arc_positions) < 20:
            self.get_logger().warn(f"Insufficient arc data for steering {math.degrees(self.current_steering_angle):.1f}°")
            return

        # Extract positions
        positions = np.array(self.arc_positions)
        x_coords = positions[:, 0]
        y_coords = positions[:, 1]

        # Check if vehicle has moved significantly
        travel_distance = np.sqrt((x_coords[-1] - x_coords[0])**2 + (y_coords[-1] - y_coords[0])**2)
        if travel_distance < 0.5:  # Minimum travel distance
            self.get_logger().warn(f"Insufficient travel distance for steering {math.degrees(self.current_steering_angle):.1f}°")
            return

        # Fit circle to path
        try:
            if self.use_least_squares:
                radius, center_x, center_y, fit_error = self.fit_circle_least_squares(x_coords, y_coords)
            else:
                radius, center_x, center_y, fit_error = self.fit_circle_algebraic(x_coords, y_coords)

            # Validate radius
            if self.min_turn_radius <= radius <= self.max_turn_radius:
                # Calculate theoretical radius using bicycle model
                if abs(self.current_steering_angle) > 1e-6:
                    theoretical_radius = self.wheelbase / abs(math.tan(self.current_steering_angle))
                else:
                    theoretical_radius = float('inf')

                # Store results
                self.calibration_results['steering_commands'].append(self.current_steering_angle)
                self.calibration_results['measured_radii'].append(radius)
                self.calibration_results['theoretical_radii'].append(theoretical_radius)

                radius_error = abs(radius - theoretical_radius) if theoretical_radius != float('inf') else 0
                self.calibration_results['radius_errors'].append(radius_error)

                self.get_logger().info(
                    f"Steering {math.degrees(self.current_steering_angle):.1f}° -> "
                    f"Measured R: {radius:.2f}m, Theoretical R: {theoretical_radius:.2f}m, "
                    f"Error: {radius_error:.2f}m"
                )
            else:
                self.get_logger().warn(
                    f"Invalid radius {radius:.2f}m for steering {math.degrees(self.current_steering_angle):.1f}°"
                )

        except Exception as e:
            self.get_logger().error(f"Failed to fit circle for steering {math.degrees(self.current_steering_angle):.1f}°: {e}")

    def fit_circle_least_squares(self, x, y):
        """Fit circle using least squares method"""
        # Convert to matrix form: Ax = b
        # (xi - cx)² + (yi - cy)² = r²
        # xi² - 2*cx*xi + cx² + yi² - 2*cy*yi + cy² = r²
        # xi² + yi² = 2*cx*xi + 2*cy*yi + (r² - cx² - cy²)

        A = np.column_stack([2*x, 2*y, np.ones(len(x))])
        b = x**2 + y**2

        # Solve using least squares
        coeffs, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)

        cx = coeffs[0]
        cy = coeffs[1]
        c = coeffs[2]

        # Calculate radius
        radius = np.sqrt(c + cx**2 + cy**2)

        # Calculate fit error
        distances = np.sqrt((x - cx)**2 + (y - cy)**2)
        fit_error = np.std(np.abs(distances - radius))

        return radius, cx, cy, fit_error

    def fit_circle_algebraic(self, x, y):
        """Fit circle using algebraic method (Pratt's method)"""
        # Center data for numerical stability
        x_mean = np.mean(x)
        y_mean = np.mean(y)
        x_centered = x - x_mean
        y_centered = y - y_mean

        # Solve circle fitting
        z = x_centered**2 + y_centered**2
        Mxy = np.mean(x_centered * y_centered)
        Mxx = np.mean(x_centered**2)
        Myy = np.mean(y_centered**2)
        Mx = np.mean(x_centered)
        My = np.mean(y_centered)
        Mz = np.mean(z)

        # Calculate circle parameters
        Mxz = np.mean(x_centered * z)
        Myz = np.mean(y_centered * z)

        # Solve 2x2 system
        det = 4 * (Mxx * Myy - Mxy**2)
        if abs(det) < 1e-10:
            raise ValueError("Degenerate circle fit")

        cx_centered = 2 * (Myy * Mxz - Mxy * Myz) / det
        cy_centered = 2 * (Mxx * Myz - Mxy * Mxz) / det

        # Convert back to original coordinates
        cx = cx_centered + x_mean
        cy = cy_centered + y_mean

        # Calculate radius
        radius = np.sqrt(cx_centered**2 + cy_centered**2 + Mz)

        # Calculate fit error
        distances = np.sqrt((x - cx)**2 + (y - cy)**2)
        fit_error = np.std(np.abs(distances - radius))

        return radius, cx, cy, fit_error

    def analyze_calibration_data(self):
        """Analyze calibration data and determine optimal steering gain and offset"""
        if len(self.calibration_results['steering_commands']) < 3:
            self.get_logger().error("Insufficient data for calibration analysis")
            return

        commanded_angles = np.array(self.calibration_results['steering_commands'])
        measured_radii = np.array(self.calibration_results['measured_radii'])
        theoretical_radii = np.array(self.calibration_results['theoretical_radii'])

        # Remove infinite radii (straight line cases)
        finite_mask = np.isfinite(theoretical_radii)
        if np.sum(finite_mask) < 3:
            self.get_logger().error("Insufficient finite radius data for calibration")
            return

        commanded_angles_finite = commanded_angles[finite_mask]
        measured_radii_finite = measured_radii[finite_mask]
        theoretical_radii_finite = theoretical_radii[finite_mask]

        # Calculate actual steering angles from measured radii
        # R = L / tan(δ) -> δ = atan(L / R)
        actual_angles = np.arctan(self.wheelbase / measured_radii_finite)

        # Preserve sign from commanded angle
        for i in range(len(actual_angles)):
            if commanded_angles_finite[i] < 0:
                actual_angles[i] = -actual_angles[i]

        try:
            # Method 1: Linear relationship with offset
            # actual_angle = gain * commanded_angle + offset
            def fit_function(params):
                gain, offset = params
                predicted_angles = gain * commanded_angles_finite + offset
                return np.sum((actual_angles - predicted_angles)**2)

            # Initial guess
            initial_guess = [self.initial_steering_gain, self.initial_steering_offset]

            # Optimize
            result = optimize.minimize(fit_function, initial_guess, method='Nelder-Mead')

            if result.success:
                optimal_gain, optimal_offset = result.x

                # Calculate goodness of fit
                predicted_angles = optimal_gain * commanded_angles_finite + optimal_offset
                ss_res = np.sum((actual_angles - predicted_angles)**2)
                ss_tot = np.sum((actual_angles - np.mean(actual_angles))**2)
                r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

                # Calculate RMSE
                rmse = np.sqrt(np.mean((actual_angles - predicted_angles)**2))

                # Store results
                self.calibration_results['optimal_gain'] = optimal_gain
                self.calibration_results['optimal_offset'] = optimal_offset
                self.calibration_results['r_squared'] = r_squared
                self.calibration_results['rmse'] = rmse

                # Publish optimal parameters
                gain_msg = Float32()
                gain_msg.data = float(optimal_gain)
                self.steering_gain_pub.publish(gain_msg)

                offset_msg = Float32()
                offset_msg.data = float(optimal_offset)
                self.steering_offset_pub.publish(offset_msg)

                self.get_logger().info("=== Steering Calibration Results ===")
                self.get_logger().info(f"Optimal Steering Gain: {optimal_gain:.4f}")
                self.get_logger().info(f"Optimal Steering Offset: {math.degrees(optimal_offset):.2f}°")
                self.get_logger().info(f"R-squared: {r_squared:.4f}")
                self.get_logger().info(f"RMSE: {math.degrees(rmse):.2f}°")

                # Quality assessment
                if r_squared > 0.95:
                    self.get_logger().info("Calibration quality: Excellent")
                elif r_squared > 0.90:
                    self.get_logger().info("Calibration quality: Good")
                elif r_squared > 0.80:
                    self.get_logger().info("Calibration quality: Fair")
                else:
                    self.get_logger().warn("Calibration quality: Poor - Consider recalibrating")

            else:
                self.get_logger().error("Optimization failed to converge")

        except Exception as e:
            self.get_logger().error(f"Failed to analyze calibration data: {e}")

    def plot_calibration_results(self):
        """Plot steering calibration results"""
        try:
            plt.figure(figsize=(15, 10))

            if len(self.calibration_results['steering_commands']) == 0:
                return

            commanded_angles = np.array(self.calibration_results['steering_commands'])
            measured_radii = np.array(self.calibration_results['measured_radii'])
            theoretical_radii = np.array(self.calibration_results['theoretical_radii'])

            # Plot 1: Steering angle vs turning radius
            plt.subplot(2, 2, 1)
            plt.scatter(np.degrees(commanded_angles), measured_radii,
                       c='blue', s=50, alpha=0.7, label='Measured Radius')
            plt.scatter(np.degrees(commanded_angles), theoretical_radii,
                       c='red', s=50, alpha=0.7, label='Theoretical Radius', marker='^')

            plt.xlabel('Commanded Steering Angle (degrees)')
            plt.ylabel('Turning Radius (m)')
            plt.title('Steering Angle vs Turning Radius')
            plt.legend()
            plt.grid(True, alpha=0.3)

            # Plot 2: Actual vs commanded steering angles
            if self.calibration_results['optimal_gain'] is not None:
                plt.subplot(2, 2, 2)

                # Calculate actual angles from measured radii
                finite_mask = np.isfinite(theoretical_radii)
                commanded_finite = commanded_angles[finite_mask]
                measured_finite = measured_radii[finite_mask]

                actual_angles = np.arctan(self.wheelbase / measured_finite)
                for i in range(len(actual_angles)):
                    if commanded_finite[i] < 0:
                        actual_angles[i] = -actual_angles[i]

                plt.scatter(np.degrees(commanded_finite), np.degrees(actual_angles),
                           c='green', s=50, alpha=0.7, label='Actual Steering')

                # Plot fitted line
                angle_range = np.linspace(np.min(commanded_finite), np.max(commanded_finite), 100)
                fitted_angles = self.calibration_results['optimal_gain'] * angle_range + \
                               self.calibration_results['optimal_offset']
                plt.plot(np.degrees(angle_range), np.degrees(fitted_angles), 'r-',
                        linewidth=2, label=f'Fitted Line (Gain: {self.calibration_results["optimal_gain"]:.3f})')

                # Perfect calibration line
                plt.plot(np.degrees(commanded_finite), np.degrees(commanded_finite), 'k--',
                        alpha=0.5, label='Perfect Calibration')

                plt.xlabel('Commanded Steering Angle (degrees)')
                plt.ylabel('Actual Steering Angle (degrees)')
                plt.title('Steering Calibration')
                plt.legend()
                plt.grid(True, alpha=0.3)

            # Plot 3: Radius errors
            plt.subplot(2, 2, 3)
            radius_errors = self.calibration_results['radius_errors']
            plt.bar(range(len(radius_errors)), radius_errors, alpha=0.7, color='orange')
            plt.xlabel('Test Index')
            plt.ylabel('Radius Error (m)')
            plt.title('Turning Radius Errors')
            plt.grid(True, alpha=0.3)

            # Plot 4: Vehicle trajectory for one test (if trajectory data available)
            plt.subplot(2, 2, 4)
            if len(self.calibration_data['position_x']) > 100:
                plt.plot(self.calibration_data['position_x'],
                        self.calibration_data['position_y'], 'b-', alpha=0.6, linewidth=1)
                plt.xlabel('X Position (m)')
                plt.ylabel('Y Position (m)')
                plt.title('Vehicle Trajectory')
                plt.grid(True, alpha=0.3)
                plt.axis('equal')

            plt.tight_layout()

            # Save plot if requested
            if self.save_calibration:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                plot_filename = os.path.join(self.output_dir, f"steering_calibration_{timestamp}.png")
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
            data_filename = os.path.join(self.output_dir, f"steering_calibration_data_{timestamp}.csv")
            df.to_csv(data_filename, index=False)

            # Save calibration results
            results_filename = os.path.join(self.output_dir, f"steering_calibration_results_{timestamp}.json")
            with open(results_filename, 'w') as f:
                # Convert numpy types for JSON serialization
                json_results = {}
                for key, value in self.calibration_results.items():
                    if isinstance(value, np.ndarray):
                        json_results[key] = value.tolist()
                    elif isinstance(value, (np.integer, np.floating)):
                        json_results[key] = float(value)
                    else:
                        json_results[key] = value

                json.dump(json_results, f, indent=2)

            # Save optimal gain and offset to config file
            if self.calibration_results['optimal_gain'] is not None:
                config_filename = os.path.join(self.output_dir, "steering_calibration_config.txt")
                with open(config_filename, 'w') as f:
                    f.write(f"# Steering Calibration Results\n")
                    f.write(f"# Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                    f.write(f"steering_gain: {self.calibration_results['optimal_gain']:.6f}\n")
                    f.write(f"steering_offset: {self.calibration_results['optimal_offset']:.6f}\n")
                    f.write(f"r_squared: {self.calibration_results['r_squared']:.4f}\n")
                    f.write(f"rmse_degrees: {math.degrees(self.calibration_results['rmse']):.3f}\n")

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

    node = SteeringCalibrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Steering Calibration Node interrupted by user")
        if node.calibration_active:
            node.stop_steering_calibration()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()