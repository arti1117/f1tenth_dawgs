#!/usr/bin/env python3
"""
Initial Pose Bridge for Cartographer Localization
Subscribes to /initialpose from RViz and sets pose via robot_localization EKF
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_localization.srv import SetPose


class InitialPoseBridge(Node):
    def __init__(self):
        super().__init__('initialpose_bridge')

        # Subscribe to initialpose from RViz
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )

        # Service client for EKF pose setting
        self.set_pose_client = self.create_client(
            SetPose,
            '/set_pose'
        )

        self.get_logger().info('InitialPose Bridge Node Started')
        self.get_logger().info('Listening for initial pose on /initialpose')
        self.get_logger().info('Will forward to /set_pose service (robot_localization EKF)')

    def initialpose_callback(self, msg):
        """Callback when initial pose is received from RViz"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Received initial pose from RViz')
        self.get_logger().info(f'Position: x={msg.pose.pose.position.x:.3f}, '
                             f'y={msg.pose.pose.position.y:.3f}, '
                             f'z={msg.pose.pose.position.z:.3f}')
        self.get_logger().info(f'Orientation: x={msg.pose.pose.orientation.x:.3f}, '
                             f'y={msg.pose.pose.orientation.y:.3f}, '
                             f'z={msg.pose.pose.orientation.z:.3f}, '
                             f'w={msg.pose.pose.orientation.w:.3f}')

        # Set pose via EKF
        self.set_ekf_pose(msg)

    def set_ekf_pose(self, pose_msg):
        """Set EKF pose via robot_localization service"""

        if not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('/set_pose service not available!')
            self.get_logger().error('Make sure robot_localization EKF node is running')
            return

        self.get_logger().info('Calling /set_pose service...')

        request = SetPose.Request()
        request.pose = pose_msg

        future = self.set_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.done():
            try:
                response = future.result()
                self.get_logger().info('✓ Initial pose set successfully!')
                self.get_logger().info('EKF and Cartographer should now use this pose')
                self.get_logger().info('=' * 60)
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
        else:
            self.get_logger().error('Service call timed out!')
            self.get_logger().error('=' * 60)


def main(args=None):
    rclpy.init(args=args)

    node = InitialPoseBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down initialpose bridge')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
