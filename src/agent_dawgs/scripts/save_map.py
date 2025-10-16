#!/usr/bin/env python3
"""
Map Saver Node for Cartographer
Subscribes to nav_msgs/OccupancyGrid and saves map to PGM/YAML format
Compatible with ROS2 nav2_map_server format
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import yaml
from PIL import Image
from datetime import datetime
import argparse
import os
import sys


class MapSaver(Node):
    def __init__(self, map_name, output_dir, map_topic='/map'):
        super().__init__('map_saver')

        self.map_name = map_name
        self.output_dir = output_dir
        self.map_topic = map_topic
        self.map_received = False
        self.occupancy_grid = None

        # Subscribe to map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        self.get_logger().info(f'Waiting for map on topic: {self.map_topic}')

    def map_callback(self, msg):
        """Callback when map is received"""
        if not self.map_received:
            self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m/cell')
            self.occupancy_grid = msg
            self.map_received = True
            self.save_map()

    def save_map(self):
        """Save occupancy grid to PGM and YAML files"""
        if self.occupancy_grid is None:
            self.get_logger().error('No map data to save!')
            return False

        # Create output directory if it doesn't exist
        os.makedirs(self.output_dir, exist_ok=True)

        # Generate timestamp-based filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        base_filename = f"{self.map_name}_{timestamp}"
        pgm_path = os.path.join(self.output_dir, f"{base_filename}.pgm")
        yaml_path = os.path.join(self.output_dir, f"{base_filename}.yaml")
        png_path = os.path.join(self.output_dir, f"{base_filename}.png")

        # Extract map data
        width = self.occupancy_grid.info.width
        height = self.occupancy_grid.info.height
        resolution = self.occupancy_grid.info.resolution
        origin = self.occupancy_grid.info.origin

        # Convert occupancy grid data to numpy array
        # ROS occupancy values: -1 (unknown), 0-100 (free to occupied)
        # PGM values: 0 (occupied), 254 (free), 205 (unknown)
        data = np.array(self.occupancy_grid.data).reshape((height, width))

        # Convert to image format (0-255)
        image_data = np.zeros((height, width), dtype=np.uint8)

        # Map values:
        # -1 (unknown) -> 205 (gray)
        # 0 (free) -> 254 (white)
        # 100 (occupied) -> 0 (black)
        # Linear interpolation for values in between
        for i in range(height):
            for j in range(width):
                value = data[i, j]
                if value == -1:
                    image_data[i, j] = 205  # Unknown
                elif value >= 0 and value <= 100:
                    # Scale: 0 (free) -> 254, 100 (occupied) -> 0
                    image_data[i, j] = int(254 - (value / 100.0) * 254)
                else:
                    image_data[i, j] = 205  # Treat invalid values as unknown

        # Flip vertically (ROS uses bottom-left origin, images use top-left)
        image_data = np.flipud(image_data)

        # Save as PGM (required for nav2_map_server)
        img = Image.fromarray(image_data, mode='L')
        img.save(pgm_path)
        self.get_logger().info(f'Saved PGM map: {pgm_path}')

        # Save as PNG (for easy visualization)
        img.save(png_path)
        self.get_logger().info(f'Saved PNG map: {png_path}')

        # Create YAML metadata file
        yaml_data = {
            'image': f"{base_filename}.pgm",
            'resolution': float(resolution),
            'origin': [float(origin.position.x),
                      float(origin.position.y),
                      float(origin.position.z)],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
            'mode': 'trinary'
        }

        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)

        self.get_logger().info(f'Saved YAML metadata: {yaml_path}')
        self.get_logger().info(f'Map saved successfully as {base_filename}')

        return True


def main(args=None):
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Save ROS2 nav_msgs/OccupancyGrid map to file')
    parser.add_argument('map_name',
                       nargs='?',
                       default='map',
                       help='Base name for saved map files (default: map)')
    parser.add_argument('--topic',
                       default='/map',
                       help='Map topic to subscribe to (default: /map)')
    parser.add_argument('--output-dir',
                       default='/home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps',
                       help='Directory to save map files')

    # Parse known args (ignore ROS2 args)
    parsed_args, unknown = parser.parse_known_args()

    # Initialize ROS2
    rclpy.init(args=args)

    # Create map saver node
    map_saver = MapSaver(
        map_name=parsed_args.map_name,
        output_dir=parsed_args.output_dir,
        map_topic=parsed_args.topic
    )

    # Spin until map is received and saved
    try:
        while rclpy.ok() and not map_saver.map_received:
            rclpy.spin_once(map_saver, timeout_sec=1.0)

        if map_saver.map_received:
            map_saver.get_logger().info('Map saved successfully. Shutting down.')
        else:
            map_saver.get_logger().warn('No map received. Shutting down.')

    except KeyboardInterrupt:
        map_saver.get_logger().info('Interrupted by user')

    finally:
        map_saver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
