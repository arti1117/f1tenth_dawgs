# VESC Steering Tuning Guide

## Steering Over-Reaction Problem

If steering reacts too aggressively during mapping or driving, adjust these parameters in `vesc.yaml`.

## Parameters to Adjust

### 1. Steering Gain (`steering_angle_to_servo_gain`)

Controls steering sensitivity:
- **Current**: `-1.2135` (high sensitivity)
- **Moderate**: `-0.9` to `-1.0`
- **Gentle**: `-0.6` to `-0.8`

**Formula**: `servo_value = gain * steering_angle + offset`

Start with `-0.9` and adjust based on testing.

### 2. Max Servo Speed (`max_servo_speed`)

Limits how fast steering can change (radians/second):
- **Current**: `3.2` rad/s (~183 deg/s - very fast)
- **Moderate**: `2.0` rad/s (~115 deg/s)
- **Gentle**: `1.5` rad/s (~86 deg/s)

For mapping, use 1.5-2.0 for smoother motion.

### 3. Servo Acceleration (`max_acceleration`)

Limits how quickly steering accelerates:
- **Current**: `2.5` m/s²
- **Gentler**: `1.5` to `2.0` m/s²

## Recommended Settings for Mapping

```yaml
# In vesc.yaml
steering_angle_to_servo_gain: -0.9      # Reduced from -1.2135
steering_angle_to_servo_offset: 0.5478  # Keep same
max_servo_speed: 1.8                     # Reduced from 3.2
max_acceleration: 2.0                    # Reduced from 2.5
```

## Testing Procedure

1. Edit `/src/base_system/f1tenth_system/f1tenth_stack/config/vesc.yaml`
2. Rebuild: `colcon build --packages-select f1tenth_stack`
3. Source: `source install/setup.bash`
4. Test with teleop first before mapping
5. Adjust values incrementally if needed

## Fine-Tuning Tips

- **Too aggressive**: Decrease gain magnitude (e.g., -0.9 → -0.7)
- **Too sluggish**: Increase gain magnitude (e.g., -0.9 → -1.1)
- **Jerky motion**: Decrease `max_servo_speed`
- **Too slow to respond**: Increase `max_servo_speed`

## Safety Notes

- Always test in open area first
- Keep emergency stop ready
- Start with gentler settings and increase if needed
- `servo_min: 0.08` and `servo_max: 0.92` prevent mechanical damage
