#!/usr/bin/env python3
"""
Offline trajectory generator that creates global waypoints JSON from existing PNG/YAML map files.
No ROS topics (odom, map) required - works directly with map files.

Usage:
    python3 offline_trajectory_generator.py --map_name <map_name> --map_dir <path_to_maps>
"""

import os
import sys
import argparse
from pathlib import Path
from typing import Optional

from global_planner_logic import GlobalPlannerLogic


class OfflineTrajectoryGenerator:
    """
    Generates global trajectory JSON files from existing PNG/YAML maps without ROS topics.
    """

    def __init__(self,
                 map_name: str,
                 map_dir: str,
                 safety_width: float = 0.25,
                 safety_width_sp: float = 0.20,
                 reverse_mapping: bool = False,
                 show_plots: bool = True):
        """
        Initialize offline trajectory generator.

        Args:
            map_name: Name of the map (without extension)
            map_dir: Full path to directory containing map files
            safety_width: Safety width for IQP trajectory (meters)
            safety_width_sp: Safety width for shortest path trajectory (meters)
            reverse_mapping: Whether to reverse the raceline direction
            show_plots: Whether to show matplotlib plots during generation
        """
        self.map_name = map_name
        self.map_dir = map_dir
        self.safety_width = safety_width
        self.safety_width_sp = safety_width_sp
        self.reverse_mapping = reverse_mapping
        self.show_plots = show_plots

        # Get config path for trajectory optimizer
        # Assumes this script is in global_planner package
        script_dir = Path(__file__).parent
        config_dir = script_dir.parent / 'config' / 'global_planner'

        if not config_dir.exists():
            print(f"Warning: Config directory not found at {config_dir}")
            print("Using current directory as fallback")
            config_dir = Path.cwd()

        self.input_path = str(config_dir)

        # Verify map files exist
        self._verify_map_files()

    def _verify_map_files(self):
        """Verify that required map files exist."""
        map_yaml = os.path.join(self.map_dir, f"{self.map_name}.yaml")
        map_png = os.path.join(self.map_dir, f"{self.map_name}.png")

        if not os.path.exists(map_yaml):
            raise FileNotFoundError(f"Map YAML not found: {map_yaml}")

        if not os.path.exists(map_png):
            raise FileNotFoundError(f"Map PNG not found: {map_png}")

        print(f"✓ Found map files:")
        print(f"  YAML: {map_yaml}")
        print(f"  PNG:  {map_png}")

    def generate(self) -> bool:
        """
        Generate global trajectory JSON from PNG/YAML map files.

        Returns:
            bool: True if successful, False otherwise
        """
        print(f"\n{'='*60}")
        print(f"Offline Trajectory Generation")
        print(f"{'='*60}")
        print(f"Map name: {self.map_name}")
        print(f"Map dir:  {self.map_dir}")
        print(f"Safety width (IQP): {self.safety_width}m")
        print(f"Safety width (SP):  {self.safety_width_sp}m")
        print(f"Reverse mapping: {self.reverse_mapping}")
        print(f"{'='*60}\n")

        try:
            # Create GlobalPlannerLogic instance in offline mode
            logic = GlobalPlannerLogic(
                safety_width_=self.safety_width,
                safety_width_sp_=self.safety_width_sp,
                occupancy_grid_thresh_=50,  # Not used in offline mode
                map_editor_mode_=False,
                create_map_=False,  # Important: We're using existing map
                map_name_=self.map_name,
                map_dir_=self.map_dir,
                finish_script_path_="",  # Not needed for offline
                input_path_=self.input_path,
                show_plots_=self.show_plots,
                filter_kernel_size_=3,
                required_laps_=0,
                reverse_mapping_=self.reverse_mapping,
                loginfo_=self._log_info,
                logwarn_=self._log_warn,
                logerror_=self._log_error
            )

            # Run the trajectory computation
            # This will read PNG/YAML and generate JSON
            success, result_map_name = logic.global_plan_logic()

            if success:
                json_path = os.path.join(self.map_dir, "global_waypoints.json")
                print(f"\n{'='*60}")
                print(f"✓ SUCCESS!")
                print(f"{'='*60}")
                print(f"Generated trajectory for: {result_map_name}")
                print(f"Output JSON: {json_path}")
                print(f"{'='*60}\n")
                return True
            else:
                print(f"\n{'='*60}")
                print(f"✗ FAILED")
                print(f"{'='*60}")
                print(f"Could not generate trajectory")
                print(f"{'='*60}\n")
                return False

        except Exception as e:
            print(f"\n{'='*60}")
            print(f"✗ ERROR")
            print(f"{'='*60}")
            print(f"Exception during trajectory generation:")
            print(f"  {type(e).__name__}: {e}")
            print(f"{'='*60}\n")
            import traceback
            traceback.print_exc()
            return False

    def _log_info(self, msg: str):
        """Log info message."""
        print(f"[INFO] {msg}")

    def _log_warn(self, msg: str):
        """Log warning message."""
        print(f"[WARN] {msg}", file=sys.stderr)

    def _log_error(self, msg: str):
        """Log error message."""
        print(f"[ERROR] {msg}", file=sys.stderr)


def main():
    """Main entry point for offline trajectory generation."""
    parser = argparse.ArgumentParser(
        description="Generate global trajectory JSON from PNG/YAML map files (offline mode)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate trajectory for map in stack_master/maps/
  python3 offline_trajectory_generator.py \\
      --map_name hangar_1905_v0 \\
      --map_dir /path/to/stack_master/maps/hangar_1905_v0

  # With custom safety widths
  python3 offline_trajectory_generator.py \\
      --map_name my_track \\
      --map_dir /path/to/maps/my_track \\
      --safety_width 0.30 \\
      --safety_width_sp 0.25

  # Reverse direction
  python3 offline_trajectory_generator.py \\
      --map_name my_track \\
      --map_dir /path/to/maps/my_track \\
      --reverse
        """
    )

    parser.add_argument(
        '--map_name',
        type=str,
        required=True,
        help='Name of the map (without .png/.yaml extension)'
    )

    parser.add_argument(
        '--map_dir',
        type=str,
        required=True,
        help='Full path to directory containing map PNG and YAML files'
    )

    parser.add_argument(
        '--safety_width',
        type=float,
        default=0.25,
        help='Safety width for IQP trajectory in meters (default: 0.25)'
    )

    parser.add_argument(
        '--safety_width_sp',
        type=float,
        default=0.20,
        help='Safety width for shortest path trajectory in meters (default: 0.20)'
    )

    parser.add_argument(
        '--reverse',
        action='store_true',
        help='Reverse the raceline direction'
    )

    parser.add_argument(
        '--no-plots',
        action='store_true',
        help='Disable matplotlib plots during generation'
    )

    args = parser.parse_args()

    # Validate map directory exists
    if not os.path.isdir(args.map_dir):
        print(f"Error: Map directory does not exist: {args.map_dir}")
        sys.exit(1)

    # Create generator
    generator = OfflineTrajectoryGenerator(
        map_name=args.map_name,
        map_dir=args.map_dir,
        safety_width=args.safety_width,
        safety_width_sp=args.safety_width_sp,
        reverse_mapping=args.reverse,
        show_plots=not args.no_plots
    )

    # Generate trajectory
    success = generator.generate()

    # Exit with appropriate code
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
