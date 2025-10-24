'''
Shared functions to read and write map information (global waypoints)
'''
import os
import csv
import json
import numpy as np
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields

from visualization_msgs.msg import MarkerArray
from f110_msgs.msg import WpntArray
from std_msgs.msg import String, Float32
from typing import Tuple, List, Dict


def write_global_waypoints(map_dir: str,
                           map_name: str,
                           map_info_str: str,
                           est_lap_time: Float32,
                           centerline_markers: MarkerArray,
                           centerline_waypoints: WpntArray,
                           global_traj_markers_iqp: MarkerArray,
                           global_traj_wpnts_iqp: WpntArray,
                           global_traj_markers_sp: MarkerArray,
                           global_traj_wpnts_sp: WpntArray,
                           trackbounds_markers: MarkerArray
                           ) -> None:
    '''
    Writes map information to a JSON file in the specified `map_dir` directory.
    - map_dir: Directory to save the JSON file
    - map_name: Name of the map (used for filename)
    - map_info_str
        - from topic /map_infos: python str
    - est_lap_time
        - from topic /estimated_lap_time: Float32
    - centerline_markers
        - from topic /centerline_waypoints/markers: MarkerArray
    - centerline_waypoints
        - from topic /centerline_waypoints: WpntArray
    - global_traj_markers_iqp
        - from topic /global_waypoints: MarkerArray
    - global_traj_wpnts_iqp
        - from topic /global_waypoints/markers: WpntArray
    - global_traj_markers_sp
        - from topic /global_waypoints/shortest_path: MarkerArray
    - global_traj_wpnts_sp
        - from topic /global_waypoitns/shortest_path/markers: WpntArray
    - trackbounds_markers
        - from topic /trackbounds/markers: MarkerArray
    - write_path: Full path of JSON file to write
    '''

    path = os.path.join(map_dir, f'{map_name}_wp.json')
    print(f"[INFO] WRITE_GLOBAL_WAYPOINTS: Writing global waypoints to {path}")

    # Dictionary will be converted into a JSON for serialization
    d: Dict[str, Dict] = {}
    d['map_info_str'] = {'data': map_info_str}
    d['est_lap_time'] = {'data': est_lap_time.data}
    d['centerline_markers'] = message_to_ordereddict(centerline_markers)
    d['centerline_waypoints'] = message_to_ordereddict(centerline_waypoints)
    d['global_traj_markers_iqp'] = message_to_ordereddict(global_traj_markers_iqp)
    d['global_traj_wpnts_iqp'] = message_to_ordereddict(global_traj_wpnts_iqp)
    d['global_traj_markers_sp'] = message_to_ordereddict(global_traj_markers_sp)
    d['global_traj_wpnts_sp'] = message_to_ordereddict(global_traj_wpnts_sp)
    d['trackbounds_markers'] = message_to_ordereddict(trackbounds_markers)

    # serialize
    with open(path, 'w') as f:
        json.dump(d, f)


def read_global_waypoints(map_dir: str, map_name: str = None) -> Tuple[
    String, Float32, MarkerArray, WpntArray, MarkerArray, WpntArray, MarkerArray, WpntArray, MarkerArray
]:
    '''
    Reads map information from a JSON file with path specified `map_dir`.

    Parameters
    ----------
    map_dir : str
        Directory containing the JSON file
    map_name : str, optional
        Name of the map (used for filename). If None, tries 'global_waypoints.json' as fallback.

    Outputs Message objects as follows:
    - map_info_str
        - from topic /map_infos: String
    - est_lap_time
        - from topic /estimated_lap_time: Float32
    - centerline_markers
        - from topic /centerline_waypoints/markers: MarkerArray
    - centerline_waypoints
        - from topic /centerline_waypoints: WpntArray
    - global_traj_markers_iqp
        - from topic /global_waypoints: MarkerArray
    - global_traj_wpnts_iqp
        - from topic /global_waypoints/markers: WpntArray
    - global_traj_markers_sp
        - from topic /global_waypoints/shortest_path: MarkerArray
    - global_traj_wpnts_sp
        - from topic /global_waypoitns/shortest_path/markers: WpntArray
    - trackbounds_markers
        - from topic /trackbounds/markers: MarkerArray
    '''
    # Try new naming convention first, fallback to old for backward compatibility
    if map_name:
        path = os.path.join(map_dir, f'{map_name}_wp.json')
        if not os.path.exists(path):
            # Fallback to old naming convention
            path = os.path.join(map_dir, 'global_waypoints.json')
    else:
        path = os.path.join(map_dir, 'global_waypoints.json')

    print(f"[INFO] READ_GLOBAL_WAYPOINTS: Reading global waypoints from {path}")
    # Deserialize JSON and Reconstruct the maps elements
    with open(path, 'r') as f:
        d: Dict[str, List] = json.load(f)

    map_info_str = String()
    set_message_fields(map_info_str, d['map_info_str'])

    est_lap_time = Float32()
    set_message_fields(est_lap_time, d['est_lap_time'])

    centerline_markers = MarkerArray()
    set_message_fields(centerline_markers, d['centerline_markers'])

    centerline_waypoints = WpntArray()
    set_message_fields(centerline_waypoints, d['centerline_waypoints'])

    global_traj_markers_iqp = MarkerArray()
    set_message_fields(global_traj_markers_iqp, d['global_traj_markers_iqp'])

    global_traj_wpnts_iqp = WpntArray()
    set_message_fields(global_traj_wpnts_iqp, d['global_traj_wpnts_iqp'])

    global_traj_markers_sp = MarkerArray()
    set_message_fields(global_traj_markers_sp, d['global_traj_markers_sp'])

    global_traj_wpnts_sp = WpntArray()
    set_message_fields(global_traj_wpnts_sp, d['global_traj_wpnts_sp'])

    trackbounds_markers = MarkerArray()
    set_message_fields(trackbounds_markers, d['trackbounds_markers'])

    return map_info_str, est_lap_time, \
        centerline_markers, centerline_waypoints, \
        global_traj_markers_iqp, global_traj_wpnts_iqp, \
        global_traj_markers_sp, global_traj_wpnts_sp, trackbounds_markers


def write_waypoints_to_csv(map_dir: str,
                           map_name: str,
                           global_traj_wpnts_iqp: WpntArray,
                           global_traj_wpnts_sp: WpntArray,
                           centerline_waypoints: WpntArray = None) -> None:
    '''
    Write global waypoints to CSV files for path_planner usage.

    CSV format: x, y, v, kappa
    - x, y: position in meters
    - v: velocity in m/s
    - kappa: curvature in rad/m

    Creates three CSV files:
    - {map_name}_iqp.csv: IQP optimized trajectory
    - {map_name}_sp.csv: Shortest path trajectory
    - {map_name}_center.csv: Centerline trajectory (if provided)

    Parameters
    ----------
    map_dir : str
        Directory to save CSV files
    map_name : str
        Name of the map (used for filename)
    global_traj_wpnts_iqp : WpntArray
        IQP optimized waypoints
    global_traj_wpnts_sp : WpntArray
        Shortest path waypoints
    centerline_waypoints : WpntArray, optional
        Centerline waypoints
    '''

    # Write IQP trajectory to CSV
    iqp_csv_path = os.path.join(map_dir, f'{map_name}_iqp.csv')
    print(f"[INFO] WRITE_CSV: Writing IQP waypoints to {iqp_csv_path}")

    with open(iqp_csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        # Write header
        writer.writerow(['x', 'y', 'v', 'kappa'])

        for wpnt in global_traj_wpnts_iqp.wpnts:
            x = wpnt.x_m
            y = wpnt.y_m
            v = wpnt.vx_mps
            kappa = wpnt.kappa_radpm
            writer.writerow([x, y, v, kappa])

    print(f"  ✓ Wrote {len(global_traj_wpnts_iqp.wpnts)} IQP waypoints")

    # Write shortest path trajectory to CSV
    sp_csv_path = os.path.join(map_dir, f'{map_name}_sp.csv')
    print(f"[INFO] WRITE_CSV: Writing SP waypoints to {sp_csv_path}")

    with open(sp_csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        # Write header
        writer.writerow(['x', 'y', 'v', 'kappa'])

        for wpnt in global_traj_wpnts_sp.wpnts:
            x = wpnt.x_m
            y = wpnt.y_m
            v = wpnt.vx_mps
            kappa = wpnt.kappa_radpm
            writer.writerow([x, y, v, kappa])

    print(f"  ✓ Wrote {len(global_traj_wpnts_sp.wpnts)} SP waypoints")

    # Write centerline trajectory to CSV if provided
    if centerline_waypoints is not None:
        center_csv_path = os.path.join(map_dir, f'{map_name}_center.csv')
        print(f"[INFO] WRITE_CSV: Writing centerline waypoints to {center_csv_path}")

        with open(center_csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            # Write header
            writer.writerow(['x', 'y', 'v', 'kappa'])

            for wpnt in centerline_waypoints.wpnts:
                x = wpnt.x_m
                y = wpnt.y_m
                v = wpnt.vx_mps
                kappa = wpnt.kappa_radpm
                writer.writerow([x, y, v, kappa])

        print(f"  ✓ Wrote {len(centerline_waypoints.wpnts)} centerline waypoints")

    # Print summary statistics
    iqp_velocities = np.array([wpnt.vx_mps for wpnt in global_traj_wpnts_iqp.wpnts])
    sp_velocities = np.array([wpnt.vx_mps for wpnt in global_traj_wpnts_sp.wpnts])

    print(f"[INFO] CSV SUMMARY:")
    print(f"  IQP trajectory:")
    print(f"    - Points: {len(global_traj_wpnts_iqp.wpnts)}")
    print(f"    - Velocity range: [{np.min(iqp_velocities):.2f}, {np.max(iqp_velocities):.2f}] m/s")
    print(f"    - Average velocity: {np.mean(iqp_velocities):.2f} m/s")
    print(f"  SP trajectory:")
    print(f"    - Points: {len(global_traj_wpnts_sp.wpnts)}")
    print(f"    - Velocity range: [{np.min(sp_velocities):.2f}, {np.max(sp_velocities):.2f}] m/s")
    print(f"    - Average velocity: {np.mean(sp_velocities):.2f} m/s")

    if centerline_waypoints is not None:
        center_velocities = np.array([wpnt.vx_mps for wpnt in centerline_waypoints.wpnts])
        print(f"  Centerline trajectory:")
        print(f"    - Points: {len(centerline_waypoints.wpnts)}")
        print(f"    - Velocity range: [{np.min(center_velocities):.2f}, {np.max(center_velocities):.2f}] m/s")
        print(f"    - Average velocity: {np.mean(center_velocities):.2f} m/s")
