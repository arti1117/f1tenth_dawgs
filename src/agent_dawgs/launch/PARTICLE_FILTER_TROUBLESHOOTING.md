# Particle Filter Localization Troubleshooting Guide

## Problem: Particle Filter Dies Immediately

The particle filter node was crashing on startup due to several issues.

## Root Causes

### 1. Missing EKF Node
- **Problem**: Particle filter expects `/odometry/filtered` topic
- **Solution**: Added EKF launch to provide filtered odometry
- **Launch file**: Now includes `ekf_launch.py`

### 2. Map Service Timing Issue
- **Problem**: Particle filter waits for `/map_server/map` service at startup
- **Cause**: Map server is a lifecycle node that takes time to activate
- **Solution**: Added 3-second respawn delay to particle filter node
- **Effect**: Map server activates before particle filter starts

### 3. Topic Remapping Issue
- **Problem**: Config had `odometry_topic: '/odom'` (absolute path)
- **Issue**: ROS2 remapping doesn't work with absolute paths in configs
- **Solution**: Changed to `odometry_topic: 'odom'` (relative path)
- **Launch file**: Remaps `odom` → `odometry/filtered`

## Fixed Launch File Changes

```python
# Added EKF for odometry filtering
ekf_launch = IncludeLaunchDescription(...)

# Added respawn to particle filter node
Node(
    package='particle_filter',
    executable='particle_filter',
    respawn=True,              # Restart if crashes
    respawn_delay=3.0,         # Wait 3s for map server
    remappings=[
        ('odom', 'odometry/filtered'),
    ],
)
```

## Usage

```bash
source install/setup.bash
ros2 launch agent_dawgs localization_particle_filter.launch.py
```

## Verification Commands

### Check if nodes are running:
```bash
ros2 node list | grep -E "(particle_filter|map_server|ekf)"
```

Expected output:
- `/particle_filter`
- `/map_server`
- `/ekf_filter_node`
- `/lifecycle_manager_localization`

### Check topics:
```bash
ros2 topic list | grep -E "(odom|filtered)"
```

Expected output:
- `/odom` (raw odometry from VESC)
- `/odometry/filtered` (EKF filtered odometry)
- `/pf/pose/odom` (particle filter output)

### Check map service:
```bash
ros2 service list | grep map
```

Expected output:
- `/map_server/map`
- `/map_server/load_map`
- etc.

## Setting Initial Pose

Use RViz "2D Pose Estimate" button or publish to `/initialpose`:

```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

## Troubleshooting

### Particle filter still crashes
1. Check logs: `ros2 launch agent_dawgs localization_particle_filter.launch.py 2>&1 | grep particle_filter`
2. Verify map file exists: `ls -la /home/dawgs_nx/f1tenth_dawgs/src/peripheral/maps/mohyun/mohyun_1016_m.yaml`
3. Check range_libc: `python3 -c "import range_libc; print('OK')"`

### Map service not available
```bash
# Check lifecycle state
ros2 service call /map_server/get_state lifecycle_msgs/srv/GetState

# Manually activate if needed
ros2 service call /map_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
```

### Odometry not publishing
```bash
# Check EKF node
ros2 node info /ekf_filter_node

# Check raw odom
ros2 topic echo /odom --once

# Check filtered odom
ros2 topic echo /odometry/filtered --once
```

## Configuration Files

- **Launch**: `src/agent_dawgs/launch/localization_particle_filter.launch.py`
- **Particle filter config**: `src/utilities/third_party/particle_filter/config/localize.yaml`
- **EKF config**: `src/agent_dawgs/config/ekf.yaml`
- **Map file**: `src/peripheral/maps/mohyun/mohyun_1016_m.yaml`

## Performance Tips

### GPU Acceleration
Particle filter uses `range_method: 'rmgpu'` for GPU-accelerated ray marching.

If GPU not available, change in config:
```yaml
range_method: 'pcddt'  # CPU-based method
```

### Particle Count
Reduce if too slow:
```yaml
max_particles: 2000  # Default: 4000
```

### Visualization
Disable if not needed:
```yaml
viz: 0  # Disable particle visualization
```
