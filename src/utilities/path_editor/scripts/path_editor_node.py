#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox, CheckButtons
from matplotlib.patches import Circle
import csv
import os
import sys
import yaml
from PIL import Image
from scipy.interpolate import splprep, splev

class PathEditorNode(Node):
    def __init__(self):
        super().__init__('path_editor')

        # Declare parameters
        self.declare_parameter('csv_file_path', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('map_image_path', '')
        self.declare_parameter('map_yaml_path', '')

        # Get parameters
        self.csv_file_path = self.get_parameter('csv_file_path').value
        self.frame_id = self.get_parameter('frame_id').value
        self.map_image_path = self.get_parameter('map_image_path').value
        self.map_yaml_path = self.get_parameter('map_yaml_path').value

        # Publisher for global centerline
        self.path_pub = self.create_publisher(Path, '/global_centerline', 10)

        # Waypoint data: [x, y, v, kappa]
        self.waypoints = []
        self.original_waypoints = []  # For refresh
        self.selected_idx = None
        self.smooth_editing = False  # Smooth curve editing option

        # Map data
        self.map_image = None
        self.map_origin = [0.0, 0.0, 0.0]  # [x, y, theta]
        self.map_resolution = 0.05

        # GUI components
        self.fig = None
        self.ax = None
        self.scatter = None
        self.line = None
        self.map_extent = None

        self.get_logger().info('Path Editor Node initialized')

        # Load map if specified
        if self.map_yaml_path and os.path.exists(self.map_yaml_path):
            self.load_map()

        # Load CSV if specified
        if self.csv_file_path and os.path.exists(self.csv_file_path):
            self.load_csv(self.csv_file_path)
            self.original_waypoints = [wp[:] for wp in self.waypoints]  # Deep copy
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {self.csv_file_path}')
        else:
            self.get_logger().warn('No valid CSV file specified')

        # Start GUI
        self.init_gui()

    def load_map(self):
        """Load map image and YAML metadata"""
        try:
            # Load YAML to get origin and resolution
            with open(self.map_yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)
                self.map_resolution = map_data.get('resolution', 0.05)
                self.map_origin = map_data.get('origin', [0.0, 0.0, 0.0])

            # Load image
            if self.map_image_path and os.path.exists(self.map_image_path):
                img = Image.open(self.map_image_path)
                self.map_image = np.array(img)

                # Calculate map extent in world coordinates
                height, width = self.map_image.shape[:2]
                x_min = self.map_origin[0]
                y_min = self.map_origin[1]
                x_max = x_min + width * self.map_resolution
                y_max = y_min + height * self.map_resolution
                self.map_extent = [x_min, x_max, y_min, y_max]

                self.get_logger().info(f'Loaded map: {width}x{height}, resolution={self.map_resolution}, origin={self.map_origin}')
            else:
                self.get_logger().warn(f'Map image not found: {self.map_image_path}')

        except Exception as e:
            self.get_logger().error(f'Failed to load map: {str(e)}')

    def load_csv(self, filepath):
        """Load waypoints from CSV file"""
        self.waypoints = []
        try:
            with open(filepath, 'r') as f:
                reader = csv.reader(f)
                header_skipped = False
                for row in reader:
                    # Skip header if present
                    if not header_skipped and ('x' in row[0].lower() or not row[0].replace('.','').replace('-','').isdigit()):
                        header_skipped = True
                        continue

                    if len(row) >= 2:
                        x = float(row[0])
                        y = float(row[1])
                        v = float(row[2]) if len(row) >= 3 else 1.0
                        kappa = float(row[3]) if len(row) >= 4 else 0.0
                        self.waypoints.append([x, y, v, kappa])

            self.get_logger().info(f'Successfully loaded {len(self.waypoints)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Failed to load CSV: {str(e)}')

    def save_csv(self, filepath):
        """Save waypoints to CSV file"""
        try:
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'v', 'kappa'])
                for wp in self.waypoints:
                    writer.writerow(wp)

            self.get_logger().info(f'Saved {len(self.waypoints)} waypoints to {filepath}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save CSV: {str(e)}')
            return False

    def publish_path(self):
        """Publish current waypoints as ROS Path message"""
        if not self.waypoints:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id

        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = wp[2]  # Store velocity in z
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(self.waypoints)} waypoints')

    def smooth_curve_adjustment(self, changed_idx, new_x, new_y):
        """Adjust neighboring waypoints smoothly when one point is moved"""
        if not self.smooth_editing or len(self.waypoints) < 5:
            return

        # Influence range (number of neighbors to adjust on each side)
        influence_range = 3

        # Calculate indices to adjust
        n = len(self.waypoints)
        indices = []
        weights = []

        for offset in range(-influence_range, influence_range + 1):
            if offset == 0:
                continue
            idx = (changed_idx + offset) % n
            # Gaussian weight: closer points have more influence
            weight = np.exp(-(offset**2) / (2 * (influence_range/2)**2))
            indices.append(idx)
            weights.append(weight)

        # Calculate displacement
        dx = new_x - self.waypoints[changed_idx][0]
        dy = new_y - self.waypoints[changed_idx][1]

        # Apply weighted displacement to neighbors
        for idx, weight in zip(indices, weights):
            self.waypoints[idx][0] += dx * weight
            self.waypoints[idx][1] += dy * weight

    def init_gui(self):
        """Initialize matplotlib GUI"""
        if not self.waypoints:
            self.get_logger().error('No waypoints to visualize')
            return

        plt.ion()  # Interactive mode
        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        self.fig.canvas.manager.set_window_title('F1TENTH Path Editor')

        # Plot map background if available
        if self.map_image is not None:
            self.ax.imshow(self.map_image, cmap='gray', origin='lower',
                          extent=self.map_extent, alpha=0.5, zorder=0)

        # Plot waypoints
        self.update_plot()

        # Add interactive elements
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_drag)
        self.dragging = False

        # Add control buttons (adjusted positions for more buttons)
        ax_save = plt.axes([0.05, 0.02, 0.08, 0.04])
        ax_publish = plt.axes([0.14, 0.02, 0.08, 0.04])
        ax_reload = plt.axes([0.23, 0.02, 0.08, 0.04])
        ax_refresh = plt.axes([0.32, 0.02, 0.08, 0.04])
        ax_smooth = plt.axes([0.42, 0.02, 0.15, 0.04])
        ax_x = plt.axes([0.62, 0.05, 0.12, 0.04])
        ax_y = plt.axes([0.62, 0.01, 0.12, 0.04])
        ax_v = plt.axes([0.78, 0.05, 0.12, 0.04])

        self.btn_save = Button(ax_save, 'Save')
        self.btn_publish = Button(ax_publish, 'Publish')
        self.btn_reload = Button(ax_reload, 'Reload')
        self.btn_refresh = Button(ax_refresh, 'Refresh')
        self.chk_smooth = CheckButtons(ax_smooth, ['Smooth Edit'], [self.smooth_editing])
        self.txt_x = TextBox(ax_x, 'X: ', initial='0.0')
        self.txt_y = TextBox(ax_y, 'Y: ', initial='0.0')
        self.txt_v = TextBox(ax_v, 'V (m/s): ', initial='0.0')

        self.btn_save.on_clicked(self.on_save)
        self.btn_publish.on_clicked(self.on_publish)
        self.btn_reload.on_clicked(self.on_reload)
        self.btn_refresh.on_clicked(self.on_refresh)
        self.chk_smooth.on_clicked(self.on_smooth_toggle)
        self.txt_x.on_submit(self.on_position_change)
        self.txt_y.on_submit(self.on_position_change)
        self.txt_v.on_submit(self.on_velocity_change)

        # Instructions
        title_text = 'Click/Drag waypoint to select and move | Arrow keys: navigate | +/-: velocity\n'
        title_text += 'WASD: move position (Shift+WASD for larger steps) | s: save | P: publish | R: refresh | Smooth Edit: adjust neighboring points'
        self.ax.set_title(title_text, fontsize=9, pad=10)

        # Connect keyboard events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)

        plt.show(block=False)
        self.get_logger().info('GUI initialized. Click/drag waypoints to edit.')

    def update_plot(self):
        """Update the plot with current waypoints"""
        if not self.waypoints:
            return

        self.ax.clear()

        # Redraw map background if available
        if self.map_image is not None:
            self.ax.imshow(self.map_image, cmap='gray', origin='lower',
                          extent=self.map_extent, alpha=0.5, zorder=0)

        # Convert to numpy arrays
        wps = np.array(self.waypoints)
        x, y, v = wps[:, 0], wps[:, 1], wps[:, 2]

        # Plot track as line
        self.line = self.ax.plot(x, y, 'k-', alpha=0.3, linewidth=1, label='Path', zorder=1)[0]

        # Plot waypoints with velocity color coding
        self.scatter = self.ax.scatter(x, y, c=v, cmap='RdYlGn_r', s=40,
                                      edgecolors='black', linewidths=0.5,
                                      vmin=np.min(v), vmax=np.max(v), zorder=2)

        # Highlight selected waypoint
        if self.selected_idx is not None and 0 <= self.selected_idx < len(self.waypoints):
            sel_wp = self.waypoints[self.selected_idx]
            self.ax.plot(sel_wp[0], sel_wp[1], 'ro', markersize=15,
                        markerfacecolor='none', markeredgewidth=2, zorder=3)

            # Update text boxes
            self.txt_x.set_val(f'{sel_wp[0]:.3f}')
            self.txt_y.set_val(f'{sel_wp[1]:.3f}')
            self.txt_v.set_val(f'{sel_wp[2]:.2f}')

        # Add colorbar
        cbar = plt.colorbar(self.scatter, ax=self.ax, label='Velocity (m/s)')

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc='upper right')

        self.fig.canvas.draw_idle()

    def on_click(self, event):
        """Handle mouse click/drag start to select waypoint"""
        if event.inaxes != self.ax or event.button != 1:  # Left click only
            return

        # Find nearest waypoint
        wps = np.array(self.waypoints)
        distances = np.sqrt((wps[:, 0] - event.xdata)**2 + (wps[:, 1] - event.ydata)**2)
        nearest_idx = np.argmin(distances)

        # Select if close enough (within 0.5m)
        if distances[nearest_idx] < 0.5:
            self.selected_idx = nearest_idx
            self.dragging = True
            self.get_logger().info(f'Selected waypoint {nearest_idx}: x={self.waypoints[nearest_idx][0]:.3f}, '
                                  f'y={self.waypoints[nearest_idx][1]:.3f}, v={self.waypoints[nearest_idx][2]:.2f}')
            self.update_plot()

    def on_release(self, event):
        """Handle mouse button release"""
        self.dragging = False

    def on_drag(self, event):
        """Handle mouse drag to move waypoint"""
        if not self.dragging or self.selected_idx is None or event.inaxes != self.ax:
            return

        old_x = self.waypoints[self.selected_idx][0]
        old_y = self.waypoints[self.selected_idx][1]
        new_x = event.xdata
        new_y = event.ydata

        # Update position
        self.waypoints[self.selected_idx][0] = new_x
        self.waypoints[self.selected_idx][1] = new_y

        # Apply smooth curve adjustment if enabled
        if self.smooth_editing:
            self.smooth_curve_adjustment(self.selected_idx, new_x, new_y)

        self.update_plot()

    def on_key_press(self, event):
        """Handle keyboard shortcuts"""
        if self.selected_idx is None and event.key not in ['s', 'p', 'r']:
            return

        # Determine movement step size (larger with shift key)
        move_step = 0.2 if 'shift' in event.key or event.key.isupper() else 0.05

        if event.key == 'right':
            # Arrow right: select next waypoint
            self.selected_idx = (self.selected_idx + 1) % len(self.waypoints)
            self.update_plot()
        elif event.key == 'left':
            # Arrow left: select previous waypoint
            self.selected_idx = (self.selected_idx - 1) % len(self.waypoints)
            self.update_plot()
        elif event.key == 'up' or event.key == '+':
            # Arrow up or +: increase velocity
            self.waypoints[self.selected_idx][2] += 0.1
            self.update_plot()
        elif event.key == 'down' or event.key == '-':
            # Arrow down or -: decrease velocity
            self.waypoints[self.selected_idx][2] = max(0.1, self.waypoints[self.selected_idx][2] - 0.1)
            self.update_plot()
        elif event.key.lower() == 'w':
            # W: move waypoint up (y+)
            old_x = self.waypoints[self.selected_idx][0]
            old_y = self.waypoints[self.selected_idx][1]
            self.waypoints[self.selected_idx][1] += move_step
            if self.smooth_editing:
                self.smooth_curve_adjustment(self.selected_idx, old_x, self.waypoints[self.selected_idx][1])
            self.update_plot()
            self.get_logger().info(f'Moved waypoint {self.selected_idx} up by {move_step:.3f}m')
        elif event.key.lower() == 's' and 'shift' not in event.key:
            # S: move waypoint down (y-) OR save (without shift)
            if event.key == 's':  # Lowercase s without shift
                self.on_save(None)
            else:  # Shift+S
                old_x = self.waypoints[self.selected_idx][0]
                old_y = self.waypoints[self.selected_idx][1]
                self.waypoints[self.selected_idx][1] -= move_step
                if self.smooth_editing:
                    self.smooth_curve_adjustment(self.selected_idx, old_x, self.waypoints[self.selected_idx][1])
                self.update_plot()
                self.get_logger().info(f'Moved waypoint {self.selected_idx} down by {move_step:.3f}m')
        elif event.key.lower() == 'a':
            # A: move waypoint left (x-)
            old_x = self.waypoints[self.selected_idx][0]
            old_y = self.waypoints[self.selected_idx][1]
            self.waypoints[self.selected_idx][0] -= move_step
            if self.smooth_editing:
                self.smooth_curve_adjustment(self.selected_idx, self.waypoints[self.selected_idx][0], old_y)
            self.update_plot()
            self.get_logger().info(f'Moved waypoint {self.selected_idx} left by {move_step:.3f}m')
        elif event.key.lower() == 'd':
            # D: move waypoint right (x+)
            old_x = self.waypoints[self.selected_idx][0]
            old_y = self.waypoints[self.selected_idx][1]
            self.waypoints[self.selected_idx][0] += move_step
            if self.smooth_editing:
                self.smooth_curve_adjustment(self.selected_idx, self.waypoints[self.selected_idx][0], old_y)
            self.update_plot()
            self.get_logger().info(f'Moved waypoint {self.selected_idx} right by {move_step:.3f}m')
        elif event.key == 'p':
            self.on_publish(None)
        elif event.key == 'r':
            self.on_refresh(None)

    def on_position_change(self, text):
        """Handle position text box changes"""
        if self.selected_idx is None:
            return

        try:
            x = float(self.txt_x.text)
            y = float(self.txt_y.text)
            old_x = self.waypoints[self.selected_idx][0]
            old_y = self.waypoints[self.selected_idx][1]

            self.waypoints[self.selected_idx][0] = x
            self.waypoints[self.selected_idx][1] = y

            # Apply smooth curve adjustment if enabled
            if self.smooth_editing:
                self.smooth_curve_adjustment(self.selected_idx, x, y)

            self.update_plot()
            self.get_logger().info(f'Updated waypoint {self.selected_idx} position: ({x:.3f}, {y:.3f})')
        except ValueError:
            self.get_logger().error('Invalid position input')

    def on_velocity_change(self, text):
        """Handle velocity text box changes"""
        if self.selected_idx is None:
            return

        try:
            v = float(self.txt_v.text)
            self.waypoints[self.selected_idx][2] = max(0.1, v)
            self.update_plot()
            self.get_logger().info(f'Updated waypoint {self.selected_idx} velocity: {v:.2f} m/s')
        except ValueError:
            self.get_logger().error('Invalid velocity input')

    def on_save(self, event):
        """Handle save button click"""
        if self.csv_file_path:
            if self.save_csv(self.csv_file_path):
                self.get_logger().info(f'Saved to {self.csv_file_path}')
                # Update original waypoints after save
                self.original_waypoints = [wp[:] for wp in self.waypoints]
        else:
            self.get_logger().error('No CSV file path specified')

    def on_publish(self, event):
        """Handle publish button click"""
        self.publish_path()

    def on_reload(self, event):
        """Handle reload button click - reload from disk"""
        if self.csv_file_path and os.path.exists(self.csv_file_path):
            self.load_csv(self.csv_file_path)
            self.original_waypoints = [wp[:] for wp in self.waypoints]
            self.selected_idx = None
            # Reset text boxes
            self.txt_x.set_val('0.0')
            self.txt_y.set_val('0.0')
            self.txt_v.set_val('0.0')
            self.update_plot()
            self.get_logger().info('Reloaded waypoints from CSV')

    def on_refresh(self, event):
        """Handle refresh button click - restore to original state"""
        if self.original_waypoints:
            self.waypoints = [wp[:] for wp in self.original_waypoints]
            self.selected_idx = None
            # Reset text boxes
            self.txt_x.set_val('0.0')
            self.txt_y.set_val('0.0')
            self.txt_v.set_val('0.0')
            self.update_plot()
            self.get_logger().info('Refreshed to original waypoints')
        else:
            self.get_logger().warn('No original waypoints to restore')

    def on_smooth_toggle(self, label):
        """Handle smooth editing checkbox toggle"""
        self.smooth_editing = not self.smooth_editing
        status = "enabled" if self.smooth_editing else "disabled"
        self.get_logger().info(f'Smooth editing {status}')

def main(args=None):
    rclpy.init(args=args)

    node = PathEditorNode()

    try:
        # Keep node alive while GUI is active
        plt.show(block=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
