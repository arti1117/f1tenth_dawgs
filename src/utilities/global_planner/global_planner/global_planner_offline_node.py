#!/usr/bin/env python3
"""
Offline Global Planner Node

This node generates global trajectories from existing PNG/YAML map files
WITHOUT requiring ROS topics (/map, /car_state/pose).

Usage:
    ros2 run global_planner global_planner_offline

Or with launch file:
    ros2 launch global_planner global_planner_offline.launch.py map_name:=hangar_1905_v0
"""

import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from f110_msgs.msg import WpntArray
from std_msgs.msg import String, Float32
from visualization_msgs.msg import MarkerArray

from .global_planner_utils import get_data_path
from .global_planner_logic import GlobalPlannerLogic
from .readwrite_global_waypoints import read_global_waypoints


class GlobalPlannerOffline(Node):
    """
    Offline global planner node that generates trajectories from PNG/YAML files.

    This node does NOT subscribe to /map or /car_state/pose.
    It reads existing map files and generates optimal racing trajectories.
    """

    def __init__(self):
        super().__init__('global_planner_offline_node',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # Get parameters
        self.map_name = self.get_parameter('map_name').value
        self.map_dir = self.get_parameter('map_dir').value
        self.reverse_mapping = self.get_parameter('reverse_mapping').value
        self.safety_width = self.get_parameter('safety_width').value
        self.safety_width_sp = self.get_parameter('safety_width_sp').value
        self.show_plots = self.get_parameter('show_plots').value

        self.get_logger().info("="*60)
        self.get_logger().info("Global Planner - OFFLINE MODE")
        self.get_logger().info("="*60)
        self.get_logger().info(f"Map name: {self.map_name}")
        self.get_logger().info(f"Map dir: {self.map_dir}")
        self.get_logger().info(f"Safety width (IQP): {self.safety_width}m")
        self.get_logger().info(f"Safety width (SP): {self.safety_width_sp}m")
        self.get_logger().info(f"Reverse mapping: {self.reverse_mapping}")
        self.get_logger().info("="*60)

        # Verify map files exist
        self._verify_map_files()

        # Initialize logic with offline mode (create_map=False)
        self.logic = GlobalPlannerLogic(
            safety_width_=self.safety_width,
            safety_width_sp_=self.safety_width_sp,
            occupancy_grid_thresh_=50,  # Not used in offline mode
            map_editor_mode_=False,
            create_map_=False,  # IMPORTANT: Offline mode
            map_name_=self.map_name,
            map_dir_=self.map_dir,
            finish_script_path_="",  # Not needed for offline
            input_path_=os.path.join(
                get_package_share_directory('global_planner'),
                'config',
                'global_planner'
            ),
            show_plots_=self.show_plots,
            filter_kernel_size_=3,
            required_laps_=0,  # Not used in offline mode
            reverse_mapping_=self.reverse_mapping,
            loginfo_=self.get_logger().info,
            logwarn_=self.get_logger().warn,
            logerror_=self.get_logger().error
        )

        # Publishers (same as online mode)
        self.global_waypoints_pub = self.create_publisher(
            WpntArray, '/global_waypoints', 10)
        self.centerline_waypoints_pub = self.create_publisher(
            WpntArray, '/centerline_waypoints', 10)
        self.global_waypoints_markers_pub = self.create_publisher(
            MarkerArray, '/global_waypoints/markers', 10)
        self.centerline_waypoints_markers_pub = self.create_publisher(
            MarkerArray, '/centerline_waypoints/markers', 10)
        self.track_bounds_pub = self.create_publisher(
            MarkerArray, '/trackbounds/markers', 10)
        self.shortest_path_waypoints_pub = self.create_publisher(
            WpntArray, '/global_waypoints/shortest_path', 10)
        self.shortest_path_waypoints_markers_pub = self.create_publisher(
            MarkerArray, '/global_waypoints/shortest_path/markers', 10)
        self.map_infos_pub = self.create_publisher(
            String, '/map_infos', 10)
        self.est_lap_time_pub = self.create_publisher(
            Float32, '/estimated_lap_time', 10)

        # Timer to run trajectory generation once
        self.create_timer(0.5, self.generate_trajectory_once)
        self.trajectory_generated = False

    def _verify_map_files(self):
        """Verify that required map files exist."""
        map_yaml = os.path.join(self.map_dir, f"{self.map_name}.yaml")
        map_png = os.path.join(self.map_dir, f"{self.map_name}.png")

        if not os.path.exists(map_yaml):
            self.get_logger().error(f"Map YAML not found: {map_yaml}")
            raise FileNotFoundError(f"Map YAML not found: {map_yaml}")

        if not os.path.exists(map_png):
            self.get_logger().error(f"Map PNG not found: {map_png}")
            raise FileNotFoundError(f"Map PNG not found: {map_png}")

        self.get_logger().info("✓ Map files verified")

    def generate_trajectory_once(self):
        """Generate trajectory once and then shutdown."""
        if self.trajectory_generated:
            return

        self.get_logger().info("Starting trajectory generation...")

        try:
            # Run trajectory generation (no ROS topics needed)
            success, result_map_name = self.logic.global_plan_logic()

            if success:
                self.get_logger().info("="*60)
                self.get_logger().info("✓ Trajectory generation SUCCESSFUL")
                self.get_logger().info("="*60)

                # Read and publish the generated waypoints
                self.read_and_publish()

                self.get_logger().info("Published waypoints to topics")
                self.get_logger().info("Shutting down offline planner node...")

                self.trajectory_generated = True

                # Schedule shutdown after a brief delay to ensure publishing
                self.create_timer(1.0, self.shutdown_node)

            else:
                self.get_logger().error("✗ Trajectory generation FAILED")
                self.trajectory_generated = True
                self.create_timer(1.0, self.shutdown_node)

        except Exception as e:
            self.get_logger().error(f"Exception during trajectory generation: {e}")
            import traceback
            traceback.print_exc()
            self.trajectory_generated = True
            self.create_timer(1.0, self.shutdown_node)

    def shutdown_node(self):
        """Shutdown the node."""
        self.destroy_node()
        rclpy.shutdown()

    def read_and_publish(self) -> None:
        """Read generated waypoints and publish them."""
        try:
            map_infos, est_lap_time, centerline_markers, centerline_wpnts, \
                glb_markers, glb_wpnts, \
                glb_sp_markers, glb_sp_wpnts, \
                track_bounds = read_global_waypoints(map_dir=self.map_dir, map_name=self.map_name)

            # Publish all waypoints and markers
            self.global_waypoints_pub.publish(glb_wpnts)
            self.centerline_waypoints_pub.publish(centerline_wpnts)
            self.centerline_waypoints_markers_pub.publish(centerline_markers)
            self.global_waypoints_markers_pub.publish(glb_markers)
            self.track_bounds_pub.publish(track_bounds)
            self.shortest_path_waypoints_pub.publish(glb_sp_wpnts)
            self.shortest_path_waypoints_markers_pub.publish(glb_sp_markers)
            self.map_infos_pub.publish(map_infos)
            self.est_lap_time_pub.publish(est_lap_time)

            self.get_logger().info(f"Map info: {map_infos.data}")
            self.get_logger().info(f"Estimated lap time: {est_lap_time.data:.2f}s")

        except FileNotFoundError as e:
            self.get_logger().error(f"{e}. Could not publish waypoints.")


def main(args=None):
    rclpy.init(args=args)

    try:
        planner = GlobalPlannerOffline()
        rclpy.spin(planner)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        return 1
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
