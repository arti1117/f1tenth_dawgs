#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Toggle-based joystick teleop for F1TENTH VESC control
RB button toggles control on/off
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header


class JoyToggleTeleop(Node):
    """
    Joystick teleoperation node with RB button toggle functionality
    """

    def __init__(self):
        super().__init__('joy_toggle_teleop')

        # Parameters
        self.declare_parameter('max_speed', 5.0)  # m/s
        self.declare_parameter('max_steering_angle', 0.41)  # radians
        self.declare_parameter('speed_axis', 1)  # Left stick vertical (up/down)
        self.declare_parameter('steering_axis', 0)  # Left stick horizontal (left/right)
        self.declare_parameter('toggle_button', 5)  # RB button (button index 5)
        self.declare_parameter('emergency_stop_button', 0)  # A button for emergency stop
        self.declare_parameter('deadzone', 0.1)  # Joystick deadzone

        # Get parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.speed_axis = self.get_parameter('speed_axis').value
        self.steering_axis = self.get_parameter('steering_axis').value
        self.toggle_button = self.get_parameter('toggle_button').value
        self.emergency_stop_button = self.get_parameter('emergency_stop_button').value
        self.deadzone = self.get_parameter('deadzone').value

        # State variables
        self.enabled = False  # Start with control disabled
        self.last_toggle_button_state = 0  # Track button state for edge detection
        self.current_speed = 0.0
        self.current_steering = 0.0

        # Publishers and Subscribers
        self.ackermann_pub = self.create_publisher(
            AckermannDriveStamped,
            'teleop',
            10
        )

        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Timer for publishing commands at regular rate
        self.timer = self.create_timer(0.05, self.publish_command)  # 20 Hz

        self.get_logger().info('Joy Toggle Teleop initialized')
        self.get_logger().info(f'Toggle button: {self.toggle_button} (RB)')
        self.get_logger().info(f'Control is DISABLED. Press RB to enable.')

    def joy_callback(self, msg):
        """Process joystick input"""

        # Check if we have enough buttons
        if len(msg.buttons) <= self.toggle_button:
            self.get_logger().warn(f'Not enough buttons. Need at least {self.toggle_button + 1} buttons.')
            return

        # Toggle logic - detect rising edge of button press
        current_toggle_state = msg.buttons[self.toggle_button]
        if current_toggle_state == 1 and self.last_toggle_button_state == 0:
            # Button was just pressed (rising edge)
            self.enabled = not self.enabled

            if self.enabled:
                self.get_logger().info('🟢 Control ENABLED - Robot will respond to joystick')
            else:
                self.get_logger().info('🔴 Control DISABLED - Robot stopped')
                # Reset speeds when disabled
                self.current_speed = 0.0
                self.current_steering = 0.0

        self.last_toggle_button_state = current_toggle_state

        # Emergency stop - works regardless of toggle state
        if len(msg.buttons) > self.emergency_stop_button:
            if msg.buttons[self.emergency_stop_button] == 1:
                self.enabled = False
                self.current_speed = 0.0
                self.current_steering = 0.0
                self.get_logger().warn('⚠️  EMERGENCY STOP - Control disabled')

        # Only process movement if enabled
        if self.enabled:
            # Get axis values
            if len(msg.axes) > self.speed_axis:
                speed_raw = msg.axes[self.speed_axis]
                # Apply deadzone
                if abs(speed_raw) < self.deadzone:
                    self.current_speed = 0.0
                else:
                    self.current_speed = speed_raw * self.max_speed

            if len(msg.axes) > self.steering_axis:
                steering_raw = msg.axes[self.steering_axis]
                # Apply deadzone
                if abs(steering_raw) < self.deadzone:
                    self.current_steering = 0.0
                else:
                    self.current_steering = steering_raw * self.max_steering_angle

    def publish_command(self):
        """Publish ackermann command at regular rate"""
        msg = AckermannDriveStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if self.enabled:
            msg.drive.speed = self.current_speed
            msg.drive.steering_angle = self.current_steering
        else:
            # Always send zero when disabled
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0

        self.ackermann_pub.publish(msg)

        # Debug output (optional)
        if self.enabled and (abs(self.current_speed) > 0.01 or abs(self.current_steering) > 0.01):
            self.get_logger().debug(
                f'Speed: {self.current_speed:.2f} m/s, '
                f'Steering: {self.current_steering:.2f} rad'
            )


def main(args=None):
    rclpy.init(args=args)

    try:
        node = JoyToggleTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()