# Frenet Path Velocity Visualization

## Overview

Added **color-coded velocity visualization** for frenet_path in RViz. The frenet path is now displayed with colors representing velocity instead of height (z coordinate).

## Changes Summary

### Problem
- **Before**: `pose.position.z = 0.1` (visualization height)
  - path_tracker received 0.1 as velocity
  - RViz showed frenet_path elevated at 0.1m
  - No velocity information visible

### Solution
Two separate outputs:
1. **`/frenet_path`** (nav_msgs::Path): For path_tracker control
   - `pose.position.z = velocity` from global_path
   - path_tracker reads accurate velocity information

2. **`/frenet_path_velocity_markers`** (MarkerArray): For RViz visualization
   - Color-coded LINE_STRIP markers
   - Blue (slow) → Green (medium) → Red (fast)
   - z = 0.05m (slightly above track level)

---

## Implementation Details

### 1. Velocity from Global Path

**`getVelocityAtS(double s_query)` function** (line 727-794):

**Purpose**: Get velocity from global_path at given s (arc length) coordinate

**Algorithm**:
```cpp
1. Calculate accumulated s for each waypoint in ref_path_
2. Detect closed loop (first/last point < 2m apart)
3. Wrap s_query if closed loop
4. Find two closest waypoints
5. Linear interpolation:
   velocity = v_lower + t * (v_upper - v_lower)
```

**Example**:
```
Global path:
s=0m   → v=3.0 m/s
s=5m   → v=4.0 m/s
s=10m  → v=5.0 m/s

Query s=7.5m:
→ Between s=[5m, 10m]
→ t = (7.5 - 5.0) / (10.0 - 5.0) = 0.5
→ v = 4.0 + 0.5 * (5.0 - 4.0) = 4.5 m/s ✅
```

---

### 2. Modified visualizeFrenetPath()

**Line 796-823**: Updated to use global_path velocity

**Before**:
```cpp
double velocity = (i < path.v.size()) ? path.v[i] : 0.0;
pose.pose.position.z = velocity;
```

**After**:
```cpp
// Get velocity from global_path at same s coordinate
double s_coord = (i < path.s.size()) ? path.s[i] : 0.0;
double velocity = getVelocityAtS(s_coord);
pose.pose.position.z = velocity;
```

**Advantage**:
- frenet_path velocity synchronized with global_path
- Obstacle avoidance trajectory maintains speed profile
- Consistent velocity planning across all paths

---

### 3. New Visualization Function

**`visualizeFrenetPathVelocity(const FrenetTraj& path)` function** (line 825-900)

**Purpose**: Create color-coded MarkerArray for RViz visualization

**Features**:
- **Color gradient**: Blue → Green → Red
- **Line width**: 0.12m (slightly thinner than global_path's 0.15m)
- **Elevation**: z = 0.05m (slightly above track level)
- **Transparency**: alpha = 0.9 (slightly more opaque than global_path)

**Color Mapping**:
```cpp
vel_ratio = (velocity - min_vel) / (max_vel - min_vel)

if (vel_ratio < 0.5) {
    // Blue → Green (slow to medium)
    r = 0.0
    g = vel_ratio * 2.0
    b = 1.0 - vel_ratio * 2.0
} else {
    // Green → Red (medium to fast)
    r = (vel_ratio - 0.5) * 2.0
    g = 1.0 - (vel_ratio - 0.5) * 2.0
    b = 0.0
}
```

**Example**:
```
min_vel = 2.0 m/s
max_vel = 6.0 m/s

velocity = 2.0 m/s → vel_ratio = 0.0 → Blue   (0.0, 0.0, 1.0)
velocity = 4.0 m/s → vel_ratio = 0.5 → Green  (0.0, 1.0, 0.0)
velocity = 6.0 m/s → vel_ratio = 1.0 → Red    (1.0, 0.0, 0.0)
```

---

## Integration

### Publisher Setup

**Line 112-113**: Added new publisher
```cpp
pub_frenet_velocity_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/frenet_path_velocity_markers", viz_qos);
```

**Line 1000**: Added member variable
```cpp
rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_frenet_velocity_markers_;
```

---

### Planning Loop Integration

**Line 619-620**: Call both visualization functions
```cpp
if (get_parameter("visualize_paths").as_bool() && best_frenet) {
    visualizeFrenetPath(*best_frenet);              // For path_tracker (with velocity in z)
    visualizeFrenetPathVelocity(*best_frenet);      // For RViz (color-coded)
}
```

---

## RViz Setup

### 1. Add MarkerArray Display

```
RViz → Add → MarkerArray
Topic: /frenet_path_velocity_markers
```

### 2. Optional: Hide Original frenet_path

If you only want color visualization:
```
Uncheck /frenet_path in RViz
```

Or keep both:
```
/frenet_path: White line (position reference)
/frenet_path_velocity_markers: Colored line (velocity visualization)
```

---

## Topics Summary

| Topic | Type | Purpose | Content |
|-------|------|---------|---------|
| `/frenet_path` | nav_msgs/Path | path_tracker control | `pose.z = velocity` from global_path |
| `/frenet_path_velocity_markers` | MarkerArray | RViz visualization | Color-coded LINE_STRIP markers |
| `/global_centerline` | nav_msgs/Path | Global reference | `pose.z = velocity` from CSV |
| `/global_path_velocity_markers` | MarkerArray | RViz visualization | Color-coded global path |

---

## Color Interpretation

### In Straight Sections
- **Red**: High speed (e.g., 5-8 m/s)
- Fast acceleration out of corners
- Maximum speed on straights

### In Corners
- **Blue**: Low speed (e.g., 2-3 m/s)
- Braking for corner entry
- Reduced speed for cornering

### Transitions
- **Green**: Medium speed (e.g., 3-5 m/s)
- Corner exit acceleration
- Speed transitions

---

## Obstacle Avoidance

**Important**: When frenet_path deviates laterally (d offset) to avoid obstacles:
- **s coordinate remains the same**
- **Velocity is determined by s, not d**
- **Color remains consistent with global_path at that s position**

**Example**:
```
Global path at s=50m: v=4.0 m/s (Green)

Obstacle detected!
→ frenet_path deviates: d = +0.5m
→ Still at s=50m
→ velocity = getVelocityAtS(50m) = 4.0 m/s
→ Color: Green (same as global_path) ✅
```

This ensures velocity planning consistency regardless of lateral deviations.

---

## Advantages

### 1. **Visual Feedback**
- Instantly see velocity profile on frenet_path
- Identify slow corners and fast sections
- Validate speed optimization results

### 2. **Debugging**
- Verify velocity propagation from global_path
- Check if path_tracker receives correct velocities
- Identify velocity discontinuities

### 3. **Performance Analysis**
- Compare different velocity profiles
- Optimize corner entry/exit speeds
- Validate friction circle constraints

### 4. **Consistency**
- frenet_path velocity always matches global_path
- Obstacle avoidance maintains speed profile
- No velocity information loss

---

## Testing

### 1. Verify Velocity Propagation

```bash
# Terminal 1: Run path_planner
ros2 launch path_planner path_planner.launch.py

# Terminal 2: Echo frenet_path
ros2 topic echo /frenet_path --field poses[0].pose.position.z
# Should show velocity values (e.g., 3.5, 4.2, 5.0)
# NOT 0.1 or 0.15!
```

### 2. Visual Inspection in RViz

**Expected**:
- **Straight sections**: Red/Orange (high speed)
- **Corner entry**: Blue (low speed)
- **Corner exit**: Green → Red (acceleration)
- **Closed loop**: Smooth color transition at start/finish line

### 3. Compare with Global Path

```bash
# Add both markers in RViz
/global_path_velocity_markers
/frenet_path_velocity_markers

# Observation:
- frenet_path should follow global_path color pattern
- Lateral deviations (obstacle avoidance) maintain color
```

---

## Performance Impact

**Computational Cost**: Minimal
- `getVelocityAtS()`: O(n) per call
- Called once per frenet trajectory point (~60 points)
- Total: ~60 × O(n) where n = ref_path size (~1000)
- **Impact**: < 1ms additional per planning cycle

**Memory**: Negligible
- Temporary velocity vector: ~60 doubles (~500 bytes)
- Marker array: ~60 markers × ~200 bytes ≈ 12 KB

**Visualization Rate**:
- Published at same rate as frenet_path (~20-50 Hz)
- No additional load on planning loop

---

## Troubleshooting

### Issue: No colors in RViz

**Solution**:
```bash
# Check if markers are published
ros2 topic list | grep frenet_path_velocity_markers

# Echo markers
ros2 topic echo /frenet_path_velocity_markers --once

# Verify topic in RViz
Add → MarkerArray → /frenet_path_velocity_markers
```

---

### Issue: All markers are same color

**Possible causes**:
1. min_vel == max_vel (constant velocity in global_path)
2. Velocity normalization issue

**Debug**:
```bash
# Check global_path velocities
ros2 topic echo /global_centerline --field poses[0].pose.position.z

# Should show varying velocities
```

---

### Issue: Colors don't match expected speeds

**Solution**:
- Colors are **relative** to min/max velocity in the path
- Blue = slowest velocity in this specific path
- Red = fastest velocity in this specific path
- Not absolute velocity values

---

## Files Modified

### Source File
`src/controller/path_planner/src/path_planner_node.cpp`:
- **Line 112-113**: Added `pub_frenet_velocity_markers_` publisher
- **Line 620**: Call `visualizeFrenetPathVelocity()` in planning loop
- **Line 727-794**: Added `getVelocityAtS()` helper function
- **Line 796-823**: Modified `visualizeFrenetPath()` to use global_path velocity
- **Line 825-900**: Added `visualizeFrenetPathVelocity()` for color visualization
- **Line 1000**: Added publisher member variable

---

## Build Status

**Package**: `path_planner`
**Build Time**: 1 minute 49 seconds
**Status**: ✅ **SUCCESS**

No errors or warnings.

---

## Usage Example

### Launch Stack
```bash
# Terminal 1: Localization
ros2 launch agent_dawgs localization_particle_filter.launch.py

# Terminal 2: Path Planner
ros2 launch path_planner path_planner.launch.py

# Terminal 3: Path Tracker
ros2 launch path_tracker path_tracker.launch.py

# RViz: Open and add marker displays
```

### Expected Behavior

**In RViz**:
1. Global path: Colored lines showing speed profile
2. Frenet path: Colored lines following global path pattern
3. When avoiding obstacles: Colors remain consistent with s position
4. Corner entry: Blue segments
5. Straights: Red segments
6. Transitions: Green segments

**In path_tracker logs**:
```
PATH_CALLBACK: Point[0]: pose.z=3.50 → pt.v=3.50 m/s
PATH_CALLBACK: Point[1]: pose.z=4.20 → pt.v=4.20 m/s
PATH_CALLBACK: Point[2]: pose.z=5.10 → pt.v=5.10 m/s
```

**In drive commands**:
```bash
ros2 topic echo /drive --field speed
# Should show velocities matching global_path (e.g., 3.5, 4.2, 5.1)
# NOT 0.1 or 0.15!
```

---

## Future Enhancements

### 1. Absolute Velocity Colors
Current: Relative to min/max in current path
Enhancement: Fixed color scale (e.g., 0-8 m/s)

```cpp
// Instead of normalizing to [0, 1]:
vel_ratio = std::min(1.0, velocity / 8.0);  // 8 m/s = red
```

### 2. Velocity Text Labels
Add text markers showing actual velocity values

```cpp
text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
text_marker.text = std::to_string(velocity) + " m/s";
```

### 3. Acceleration Visualization
Color code by acceleration instead of velocity

```cpp
double accel = (v[i+1] - v[i]) / dt;
// Red = high accel, Blue = braking
```

### 4. Combined Visualization
Show both velocity and lateral offset

```cpp
// Line thickness = velocity
// Color = lateral offset (d coordinate)
```

---

## Summary

✅ **Frenet path velocity visualization implemented**
✅ **Color-coded markers: Blue (slow) → Green (medium) → Red (fast)**
✅ **Velocity synchronized with global_path via s coordinate**
✅ **Separate topics for control and visualization**
✅ **Maintains consistency during obstacle avoidance**
✅ **Minimal performance impact**

The frenet_path now provides clear visual feedback of velocity planning in RViz while maintaining accurate velocity data for path_tracker control.
