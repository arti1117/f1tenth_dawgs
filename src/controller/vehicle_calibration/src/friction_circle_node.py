#!/usr/bin/env python3
"""
Friction Circle Node for F1TENTH Vehicle Calibration

This node generates a friction circle test by commanding the vehicle in circular patterns
at increasing speeds to determine the maximum cornering capability and friction limits.

The friction circle represents the maximum combined longitudinal and lateral forces
that the tires can generate before losing grip.

Author: F1TENTH DAWGS
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time
import math

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist, Vector3Stamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String


class FrictionCircleNode(Node):
    def __init__(self):
        super().__init__('friction_circle_node')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_speed', 3.0),           # Maximum test speed (m/s)
                ('min_speed', 0.5),           # Minimum test speed (m/s)
                ('speed_increment', 0.2),     # Speed increment per test (m/s)
                ('test_duration', 10.0),      # Duration for each speed test (s)
                ('wheelbase', 0.33),          # Vehicle wheelbase (m)
                ('max_steering_angle', 0.4),  # Maximum steering angle (rad)
                ('data_collection_rate', 50), # Data collection rate (Hz)
                ('plot_real_time', True),     # Enable real-time plotting
                ('save_data', True),          # Save data to file
                ('output_directory', '/home/dawgs_nx/f1tenth_dawgs/data/friction_circle')
            ]
        )

        # Get parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.speed_increment = self.get_parameter('speed_increment').value
        self.test_duration = self.get_parameter('test_duration').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.data_rate = self.get_parameter('data_collection_rate').value
        self.plot_real_time = self.get_parameter('plot_real_time').value
        self.save_data = self.get_parameter('save_data').value
        self.output_dir = self.get_parameter('output_directory').value

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
            '/friction_circle/status',
            reliable_qos
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            best_effort_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/sensors/imu',
            self.imu_callback,
            best_effort_qos
        )

        self.start_sub = self.create_subscription(
            Bool,
            '/friction_circle/start',
            self.start_test_callback,
            reliable_qos
        )

        self.stop_sub = self.create_subscription(
            Bool,
            '/friction_circle/stop',
            self.stop_test_callback,
            reliable_qos
        )

        # Data storage
        self.test_data = {
            'timestamp': [],
            'speed': [],
            'steering_angle': [],
            'lateral_accel': [],
            'longitudinal_accel': [],
            'yaw_rate': [],
            'slip_angle': [],
            'friction_utilization': []
        }

        # State variables
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.current_accel_x = 0.0
        self.current_accel_y = 0.0
        self.current_yaw_rate = 0.0
        self.vehicle_velocity = [0.0, 0.0, 0.0]

        # Test control
        self.test_active = False
        self.current_test_speed = self.min_speed
        self.test_start_time = None
        self.current_phase = "idle"  # idle, testing, completed

        # Safety limits
        self.max_lateral_accel = 8.0  # m/s^2
        self.emergency_stop = False

        # Timer for test execution
        self.test_timer = self.create_timer(1.0/self.data_rate, self.test_execution_callback)

        # Real-time plotting setup
        if self.plot_real_time:
            self.setup_plotting()

        self.get_logger().info("Friction Circle Node initialized")
        self.get_logger().info(f"Speed range: {self.min_speed} - {self.max_speed} m/s")
        self.get_logger().info(f"Test duration per speed: {self.test_duration} seconds")

    def setup_plotting(self):
        """Setup real-time plotting"""
        try:
            plt.ion()
            self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 5))

            # Friction circle plot
            self.ax1.set_title('Friction Circle')
            self.ax1.set_xlabel('Longitudinal Acceleration (m/s²)')
            self.ax1.set_ylabel('Lateral Acceleration (m/s²)')
            self.ax1.grid(True)
            self.ax1.set_aspect('equal')

            # Speed vs lateral acceleration plot
            self.ax2.set_title('Speed vs Lateral Acceleration')
            self.ax2.set_xlabel('Speed (m/s)')
            self.ax2.set_ylabel('Lateral Acceleration (m/s²)')
            self.ax2.grid(True)

            # Initialize empty plots
            self.friction_plot, = self.ax1.plot([], [], 'b.', markersize=2, alpha=0.6)
            self.speed_accel_plot, = self.ax2.plot([], [], 'r.', markersize=3)

            plt.tight_layout()
            self.plot_thread = threading.Thread(target=self.update_plots, daemon=True)
            self.plot_thread.start()

        except Exception as e:
            self.get_logger().warn(f"Could not setup real-time plotting: {e}")
            self.plot_real_time = False

    def odom_callback(self, msg):
        """Odometry callback for velocity information"""
        self.vehicle_velocity[0] = msg.twist.twist.linear.x
        self.vehicle_velocity[1] = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self.current_yaw_rate = msg.twist.twist.angular.z

    def imu_callback(self, msg):
        """IMU callback for acceleration data"""
        self.current_accel_x = msg.linear_acceleration.x
        self.current_accel_y = msg.linear_acceleration.y

        # Safety check for excessive lateral acceleration
        if abs(self.current_accel_y) > self.max_lateral_accel:
            self.get_logger().warn(f"Excessive lateral acceleration: {self.current_accel_y:.2f} m/s²")
            self.emergency_stop = True

    def start_test_callback(self, msg):
        """Start friction circle test"""
        if msg.data and not self.test_active:
            self.start_friction_circle_test()

    def stop_test_callback(self, msg):
        """Stop friction circle test"""
        if msg.data:
            self.stop_friction_circle_test()

    def start_friction_circle_test(self):
        """Initialize and start the friction circle test"""
        self.get_logger().info("Starting Friction Circle Test")

        # Reset test parameters
        self.test_active = True
        self.current_test_speed = self.min_speed
        self.test_start_time = time.time()
        self.current_phase = "testing"
        self.emergency_stop = False

        # Clear previous data
        for key in self.test_data.keys():
            self.test_data[key].clear()

        # Publish status
        status_msg = String()
        status_msg.data = f"Test started - Speed: {self.current_test_speed:.1f} m/s"
        self.status_pub.publish(status_msg)

    def stop_friction_circle_test(self):
        """Stop the friction circle test"""
        self.get_logger().info("Stopping Friction Circle Test")

        self.test_active = False
        self.current_phase = "completed"
        self.emergency_stop = False

        # Send stop command
        self.send_drive_command(0.0, 0.0)

        # Save data if requested
        if self.save_data:
            self.save_test_data()

        # Generate final friction circle plot
        if len(self.test_data['lateral_accel']) > 100:
            self.generate_friction_circle_analysis()

        # Publish status
        status_msg = String()
        status_msg.data = "Test completed"
        self.status_pub.publish(status_msg)

    def test_execution_callback(self):
        """Main test execution loop"""
        if not self.test_active or self.emergency_stop:
            if self.emergency_stop:
                self.stop_friction_circle_test()
            return

        current_time = time.time()

        # Check if current speed test duration is complete
        if current_time - self.test_start_time > self.test_duration:
            # Move to next speed level
            self.current_test_speed += self.speed_increment

            if self.current_test_speed > self.max_speed:
                # Test complete
                self.stop_friction_circle_test()
                return

            # Start new speed test
            self.test_start_time = current_time
            self.get_logger().info(f"Testing at speed: {self.current_test_speed:.1f} m/s")

            # Publish status
            status_msg = String()
            status_msg.data = f"Testing speed: {self.current_test_speed:.1f} m/s"
            self.status_pub.publish(status_msg)

        # Generate circular motion command
        self.execute_circular_motion()

        # Collect data
        self.collect_data(current_time)

    def execute_circular_motion(self):
        """Execute circular motion at current test speed"""
        # Calculate steering angle for circular motion
        # Using bicycle model: R = wheelbase / tan(steering_angle)
        # For maximum lateral acceleration test, use varying steering angles

        # Time-based steering angle variation for comprehensive testing
        elapsed_time = time.time() - self.test_start_time

        # Create a pattern that tests different curvatures
        steering_frequency = 0.2  # Hz
        steering_amplitude = self.max_steering_angle * 0.8

        # Sinusoidal steering pattern with increasing amplitude
        progress = min(elapsed_time / self.test_duration, 1.0)
        current_amplitude = steering_amplitude * progress

        self.current_steering = current_amplitude * math.sin(2 * math.pi * steering_frequency * elapsed_time)

        # Send drive command
        self.send_drive_command(self.current_test_speed, self.current_steering)

    def send_drive_command(self, speed, steering_angle):
        """Send Ackermann drive command"""
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.speed = float(speed)
        drive_msg.drive.steering_angle = float(steering_angle)

        self.drive_pub.publish(drive_msg)

    def collect_data(self, timestamp):
        """Collect test data"""
        # Calculate slip angle (simplified)
        if abs(self.current_speed) > 0.1:
            slip_angle = math.atan2(self.vehicle_velocity[1], self.vehicle_velocity[0])
        else:
            slip_angle = 0.0

        # Calculate friction utilization (normalized)
        total_accel = math.sqrt(self.current_accel_x**2 + self.current_accel_y**2)
        max_friction_accel = 9.81 * 0.9  # Assume mu = 0.9
        friction_utilization = min(total_accel / max_friction_accel, 1.0)

        # Store data
        self.test_data['timestamp'].append(timestamp)
        self.test_data['speed'].append(self.current_speed)
        self.test_data['steering_angle'].append(self.current_steering)
        self.test_data['lateral_accel'].append(self.current_accel_y)
        self.test_data['longitudinal_accel'].append(self.current_accel_x)
        self.test_data['yaw_rate'].append(self.current_yaw_rate)
        self.test_data['slip_angle'].append(slip_angle)
        self.test_data['friction_utilization'].append(friction_utilization)

    def update_plots(self):
        """Update real-time plots"""
        while self.plot_real_time and rclpy.ok():
            try:
                if len(self.test_data['lateral_accel']) > 10:
                    # Update friction circle plot
                    self.friction_plot.set_data(
                        self.test_data['longitudinal_accel'],
                        self.test_data['lateral_accel']
                    )

                    # Update speed vs lateral acceleration plot
                    self.speed_accel_plot.set_data(
                        self.test_data['speed'],
                        np.abs(self.test_data['lateral_accel'])
                    )

                    # Update plot limits
                    accel_data = np.array(self.test_data['longitudinal_accel'] + self.test_data['lateral_accel'])
                    if len(accel_data) > 0:
                        accel_max = max(np.abs(accel_data)) * 1.1
                        self.ax1.set_xlim(-accel_max, accel_max)
                        self.ax1.set_ylim(-accel_max, accel_max)

                    speed_max = max(self.test_data['speed']) * 1.1 if self.test_data['speed'] else 5.0
                    lat_accel_max = max(np.abs(self.test_data['lateral_accel'])) * 1.1 if self.test_data['lateral_accel'] else 5.0
                    self.ax2.set_xlim(0, speed_max)
                    self.ax2.set_ylim(0, lat_accel_max)

                    plt.pause(0.01)

                time.sleep(0.1)

            except Exception as e:
                self.get_logger().debug(f"Plot update error: {e}")
                time.sleep(0.5)

    def save_test_data(self):
        """Save test data to file"""
        try:
            import os
            import pandas as pd

            # Create output directory
            os.makedirs(self.output_dir, exist_ok=True)

            # Create filename with timestamp
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(self.output_dir, f"friction_circle_test_{timestamp}.csv")

            # Convert to DataFrame and save
            df = pd.DataFrame(self.test_data)
            df.to_csv(filename, index=False)

            self.get_logger().info(f"Test data saved to: {filename}")

        except Exception as e:
            self.get_logger().error(f"Failed to save test data: {e}")

    def generate_friction_circle_analysis(self):
        """Generate comprehensive friction circle analysis"""
        try:
            import matplotlib.pyplot as plt
            import numpy as np

            # Create analysis plots
            fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))

            # 1. Friction Circle
            ax1.scatter(self.test_data['longitudinal_accel'],
                       self.test_data['lateral_accel'],
                       c=self.test_data['speed'],
                       cmap='viridis',
                       alpha=0.6, s=10)
            ax1.set_xlabel('Longitudinal Acceleration (m/s²)')
            ax1.set_ylabel('Lateral Acceleration (m/s²)')
            ax1.set_title('Friction Circle')
            ax1.grid(True)
            ax1.set_aspect('equal')

            # Draw theoretical friction circle
            theta = np.linspace(0, 2*np.pi, 100)
            max_accel = 9.81 * 0.9  # Assume mu = 0.9
            ax1.plot(max_accel * np.cos(theta), max_accel * np.sin(theta), 'r--',
                    linewidth=2, label='Theoretical Limit (μ=0.9)')
            ax1.legend()

            # 2. Speed vs Lateral Acceleration
            speeds = np.array(self.test_data['speed'])
            lat_accels = np.abs(np.array(self.test_data['lateral_accel']))

            ax2.scatter(speeds, lat_accels, alpha=0.6, s=10)
            ax2.set_xlabel('Speed (m/s)')
            ax2.set_ylabel('Lateral Acceleration (m/s²)')
            ax2.set_title('Speed vs Lateral Acceleration')
            ax2.grid(True)

            # 3. Friction Utilization vs Speed
            ax3.scatter(self.test_data['speed'],
                       self.test_data['friction_utilization'],
                       alpha=0.6, s=10)
            ax3.set_xlabel('Speed (m/s)')
            ax3.set_ylabel('Friction Utilization')
            ax3.set_title('Friction Utilization vs Speed')
            ax3.grid(True)
            ax3.set_ylim(0, 1.1)

            # 4. Slip Angle vs Lateral Acceleration
            ax4.scatter(np.abs(self.test_data['slip_angle']),
                       np.abs(self.test_data['lateral_accel']),
                       alpha=0.6, s=10)
            ax4.set_xlabel('Slip Angle (rad)')
            ax4.set_ylabel('Lateral Acceleration (m/s²)')
            ax4.set_title('Slip Angle vs Lateral Acceleration')
            ax4.grid(True)

            plt.tight_layout()

            # Save analysis plot
            if self.save_data:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                analysis_filename = os.path.join(self.output_dir, f"friction_circle_analysis_{timestamp}.png")
                plt.savefig(analysis_filename, dpi=300, bbox_inches='tight')
                self.get_logger().info(f"Analysis plot saved to: {analysis_filename}")

            plt.show(block=False)

        except Exception as e:
            self.get_logger().error(f"Failed to generate analysis: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = FrictionCircleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Friction Circle Node interrupted by user")
        if node.test_active:
            node.stop_friction_circle_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()