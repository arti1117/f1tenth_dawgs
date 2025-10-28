# Path Tracker Tuning Guide

Complete guide for tuning the path_tracker node for optimal performance.

## Quick Reference: Steering Response

**Problem: Slow steering response / Poor tracking**
```yaml
# Increase these:
stanley_k: 1.0              # (default: 0.5) � Higher = faster lateral error correction
steering_alpha: 0.8         # (default: 0.5) � Higher = more responsive steering

# Decrease these:
lookahead_base: 0.25        # (default: 0.3) � Lower = tighter following
lookahead_k: 0.3            # (default: 0.4) � Lower = less speed dependency

# Fix this if wrong:
k_error: -0.3               # Should be NEGATIVE to reduce lookahead on large errors
```

**Problem: Oscillation / Overshooting**
```yaml
# Decrease these:
stanley_k: 0.3              # Lower = gentler correction
steering_alpha: 0.3         # Lower = smoother steering

# Increase these:
lookahead_base: 0.4         # Higher = more predictive, less reactive
```

---

## Parameter Categories

### 1. Pure Pursuit Parameters

#### `lookahead_base` (meters)
- **What it does**: Base lookahead distance for target point selection
- **Default**: 0.3m (very aggressive for tight tracking)
- **Range**: 0.2 - 2.0m
- **Effects**:
  - **Lower (0.2-0.3m)**: Tighter path following, faster response, more oscillation
  - **Higher (0.5-1.0m)**: Smoother tracking, slower response, better stability
- **Tuning**:
  - Start with 0.3m for aggressive racing
  - Increase if oscillating or overshooting corners
  - Decrease if response is too slow

#### `lookahead_k` (dimensionless)
- **What it does**: Speed-dependent lookahead scaling (L = lookahead_base + lookahead_k � speed)
- **Default**: 0.4
- **Range**: 0.2 - 1.0
- **Effects**:
  - **Lower (0.2-0.4)**: Less speed dependency, more consistent response
  - **Higher (0.6-1.0)**: More predictive at high speeds, smoother at speed
- **Tuning**:
  - Decrease if high-speed tracking is too loose
  - Increase if vehicle is unstable at high speeds

#### `use_speed_lookahead` (boolean)
- **What it does**: Enable/disable speed-dependent lookahead
- **Default**: true
- **When to disable**:
  - Constant low-speed operation
  - When speed-dependent behavior causes issues

---

### 2. Adaptive Lookahead Parameters

#### `use_adaptive_lookahead` (boolean)
- **What it does**: Dynamically adjust lookahead based on curvature and lateral error
- **Default**: true
- **Benefits**: Better corner handling, self-correcting for errors
- **When to disable**: If behavior becomes unpredictable or unstable

#### `lookahead_min` / `lookahead_max` (meters)
- **What it does**: Bounds for adaptive lookahead adjustment
- **Default**: 0.4m - 3.0m
- **Tuning**:
  - Tighten range (0.3-1.5m) for more consistent behavior
  - Widen range for diverse track conditions

#### `k_curvature` (dimensionless)
- **What it does**: Increases lookahead in tight corners (L += k_curvature / |kappa|)
- **Default**: 0.5
- **Range**: 0.2 - 1.0
- **Effects**:
  - **Higher**: More predictive in corners, less cutting
  - **Lower**: Tighter corner following, risk of cutting

#### `k_error` (dimensionless)
- **What it does**: REDUCES lookahead when lateral error is large (L += k_error � |lateral_error|)
- **Default**: 0.3 (WRONG - should be negative!)
- **Correct value**: -0.3 to -0.5
- **Why negative**: Large lateral error � reduce lookahead � more aggressive correction
- **Effects**:
  - **More negative (-0.5)**: Faster recovery from large errors
  - **Less negative (-0.1)**: Gentler error correction

---

### 3. Stanley Controller Parameters

#### `use_stanley` (boolean)
- **What it does**: Adds Stanley controller term to Pure Pursuit for lateral error correction
- **Default**: true
- **Benefits**: Eliminates steady-state lateral error, suppresses corner cutting
- **Formula**: steering_angle += atan(stanley_k � lateral_error / speed)

#### `stanley_k` (dimensionless)
- **What it does**: Gain for Stanley lateral error correction
- **Default**: 0.5 (conservative)
- **Range**: 0.3 - 2.0
- **Effects**:
  - **Lower (0.3)**: Gentle correction, smooth but slower convergence
  - **Medium (0.5-1.0)**: Balanced correction
  - **Higher (1.5-2.0)**: Aggressive correction, fast convergence, risk of oscillation
- **Tuning for fast response**:
  - Start with 1.0
  - Increase to 1.5 if still too slow
  - Decrease if oscillating

---

### 4. Steering Filter Parameters

#### `use_steering_filter` (boolean)
- **What it does**: Applies low-pass filter to smooth steering commands
- **Default**: true
- **When to disable**: Maximum responsiveness needed, smooth paths

#### `steering_alpha` (0.0 - 1.0)
- **What it does**: Filter coefficient for exponential moving average
- **Formula**: steering = alpha � new_steering + (1-alpha) � prev_steering
- **Default**: 0.5 (balanced)
- **Range**: 0.0 - 1.0
- **Effects**:
  - **Low (0.2-0.3)**: Very smooth, filtered, slow response
  - **Medium (0.5)**: Balanced smoothness and response
  - **High (0.7-0.9)**: Responsive, less filtering, closer to raw commands
  - **1.0**: No filtering at all (not recommended)
- **Tuning for fast response**:
  - Increase to 0.7-0.8 for better responsiveness
  - Keep some filtering (< 1.0) to avoid jerky steering

---

### 5. Speed Control Parameters

#### `speed_mode` (string)
- **Options**:
  - `"default"`: Use constant `default_speed`
  - `"path_velocity"`: Use velocity from global_centerline waypoints
  - `"curvature"`: Calculate speed based on path curvature and friction
  - `"optimize"`: Minimum of path_velocity, curvature limit, and friction circle limit
- **Default**: "optimize"
- **Recommended**: "optimize" for racing, "path_velocity" for following pre-optimized paths

#### `friction_coeff` (dimensionless)
- **What it does**: Friction coefficient (�) for lateral grip calculation
- **Default**: 0.9
- **Range**: 0.7 (wet) - 1.2 (high grip)
- **Formula**: max_lateral_accel = friction_coeff � g (9.81 m/s�)

#### `max_speed_limit` / `min_speed_limit` (m/s)
- **What it does**: Hard limits on commanded speed
- **Default**: 0.5 - 8.0 m/s
- **Tuning**: Set based on vehicle capabilities and safety requirements

---

### 6. Vehicle Parameters

#### `wheelbase` (meters)
- **What it does**: Distance between front and rear axles
- **Default**: 0.33m (F1TENTH standard)
- **Critical**: Must match physical vehicle for correct steering geometry

#### `max_steering_angle` (radians)
- **What it does**: Maximum steering angle limit
- **Default**: 0.4189 rad (~24 degrees)
- **Critical**: Must match physical servo limits to prevent damage

---

### 7. Debug Mode Parameters

#### `debug_mode` (boolean)
- **What it does**: Enables reduced speed mode for safe testing
- **Default**: true
- **Set to false**: For full performance racing

#### `velocity_gain` (0.0 - 1.0)
- **What it does**: Speed scaling factor when debug_mode = true
- **Default**: 0.2 (20% speed)
- **Range**: 0.1 - 1.0
- **Usage**: Gradually increase from 0.2 � 0.5 � 1.0 during testing

---

## Tuning Workflow

### Step 1: Baseline Setup
```yaml
lookahead_base: 0.3
lookahead_k: 0.4
stanley_k: 0.5
steering_alpha: 0.5
use_adaptive_lookahead: true
use_stanley: true
use_steering_filter: true
debug_mode: true
velocity_gain: 0.2
```

### Step 2: Test and Observe
- Run on track and observe behavior
- Check for: oscillation, slow response, corner cutting, stability

### Step 3: Adjust for Fast Response
If tracking is too slow:
```yaml
# Priority 1: Increase correction gain
stanley_k: 1.0              # Was 0.5

# Priority 2: Increase steering responsiveness
steering_alpha: 0.7         # Was 0.5

# Priority 3: Fix error correction (if wrong)
k_error: -0.3               # Should be negative!

# Priority 4: Slightly reduce lookahead (if needed)
lookahead_base: 0.25        # Was 0.3
lookahead_k: 0.3            # Was 0.4
```

### Step 4: Handle Oscillation
If vehicle oscillates or overshoots:
```yaml
# Reduce correction gains
stanley_k: 0.3              # Lower than before
steering_alpha: 0.4         # More filtering

# Increase lookahead for smoother tracking
lookahead_base: 0.4         # Higher = smoother
```

### Step 5: Fine-tune Speed
```yaml
# Gradually increase speed
velocity_gain: 0.3          # Then 0.5, 0.7, 1.0
debug_mode: false           # When confident

# Adjust friction coefficient if slipping
friction_coeff: 0.8         # Lower if losing grip
```

---

## Common Issues and Solutions

### Issue 1: Vehicle cuts corners
**Cause**: Lookahead too short or insufficient Stanley correction
**Solution**:
```yaml
lookahead_base: 0.4         # Increase
stanley_k: 1.0              # Increase
k_curvature: 0.7            # Increase for more predictive corner entry
```

### Issue 2: Slow reaction to path changes
**Cause**: High filtering, low correction gains
**Solution**:
```yaml
stanley_k: 1.2              # Increase
steering_alpha: 0.8         # Increase (less filtering)
lookahead_base: 0.25        # Decrease (more reactive)
```

### Issue 3: Oscillation around path
**Cause**: Gains too high, lookahead too short
**Solution**:
```yaml
stanley_k: 0.3              # Decrease
steering_alpha: 0.3         # Decrease (more filtering)
lookahead_base: 0.5         # Increase (more predictive)
```

### Issue 4: Unstable at high speeds
**Cause**: Insufficient speed-dependent lookahead
**Solution**:
```yaml
lookahead_k: 0.6            # Increase speed dependency
lookahead_max: 2.5          # Increase upper bound
```

### Issue 5: Poor recovery from large errors
**Cause**: Positive k_error (wrong!) or too low stanley_k
**Solution**:
```yaml
k_error: -0.4               # Must be NEGATIVE!
stanley_k: 1.0              # Increase for faster correction
```

---

## Recommended Configurations

### Configuration 1: Aggressive Racing (Fast Response)
```yaml
lookahead_base: 0.25
lookahead_k: 0.3
stanley_k: 1.2
steering_alpha: 0.8
k_error: -0.4
use_adaptive_lookahead: true
use_stanley: true
velocity_gain: 1.0
```

### Configuration 2: Smooth Racing (Stable)
```yaml
lookahead_base: 0.4
lookahead_k: 0.5
stanley_k: 0.7
steering_alpha: 0.5
k_error: -0.3
use_adaptive_lookahead: true
use_stanley: true
velocity_gain: 1.0
```

### Configuration 3: Testing/Debug (Safe)
```yaml
lookahead_base: 0.5
lookahead_k: 0.4
stanley_k: 0.5
steering_alpha: 0.4
k_error: -0.3
use_adaptive_lookahead: true
use_stanley: true
debug_mode: true
velocity_gain: 0.2
```

### Configuration 4: Maximum Responsiveness (Experimental)
```yaml
lookahead_base: 0.2
lookahead_k: 0.25
stanley_k: 1.5
steering_alpha: 0.9
k_error: -0.5
use_adaptive_lookahead: true
use_stanley: true
velocity_gain: 1.0
```

---

## How to Apply Changes

### Method 1: Edit config file (requires restart)
```bash
# Edit the parameter file
nano ~/f1tenth_dawgs/src/controller/path_tracker/config/tracker_params.yaml

# Rebuild (only if source code changed, not needed for yaml)
colcon build --packages-select path_tracker

# Restart node
ros2 launch path_tracker path_tracker.launch.py
```

### Method 2: Dynamic parameter update (no restart needed)
```bash
# Update parameters on running node
ros2 param set /path_tracker_node stanley_k 1.0
ros2 param set /path_tracker_node steering_alpha 0.8
ros2 param set /path_tracker_node lookahead_base 0.25
ros2 param set /path_tracker_node k_error -0.3

# Check current values
ros2 param get /path_tracker_node stanley_k
```

### Method 3: Save current parameters to file
```bash
# Dump all current parameters
ros2 param dump /path_tracker_node > /tmp/current_tracker_params.yaml

# Review and copy to config directory if satisfied
```

---

## Performance Monitoring

### Check tracking quality
```bash
# Monitor drive commands
ros2 topic echo /drive

# Visualize in RViz
# - Check lookahead_point marker (green sphere)
# - Distance from path indicates error
# - Smooth steering = good tracking
```

### Debug with log messages
Set higher log level in code or via parameter for detailed tracking info.

---

## Advanced Tuning Tips

1. **Test on different track sections**:
   - Tight corners: May need higher stanley_k or lower lookahead
   - Straights: May need higher lookahead_k for stability
   - Chicanes: Test oscillation behavior

2. **Tune progressively**:
   - Start conservative (smooth config)
   - Gradually increase aggressiveness
   - Find the limit, then back off 10-20%

3. **Balance speed and steering**:
   - Faster response needs careful speed control
   - Consider friction_coeff and speed limits

4. **Monitor servo health**:
   - Very aggressive steering can stress servos
   - Add filtering if servo gets hot or jittery

5. **Path quality matters**:
   - Smooth paths need less aggressive tracking
   - Noisy/irregular paths need more filtering

---

## Quick Fix for Current Issue

Based on your description of slow steering response, apply these changes immediately:

```bash
# Increase Stanley controller gain for faster lateral correction
ros2 param set /path_tracker_node stanley_k 1.2

# Increase steering responsiveness (less filtering)
ros2 param set /path_tracker_node steering_alpha 0.8

# Fix k_error sign if wrong (check current value first)
ros2 param get /path_tracker_node k_error
# If positive, set to negative:
ros2 param set /path_tracker_node k_error -0.3

# Optionally reduce lookahead for tighter following
ros2 param set /path_tracker_node lookahead_base 0.25
```

Test the changes and adjust further based on observed behavior.


--- KOR guide
● 완료! TUNING_GUIDE.md 생성 완료

  src/controller/path_tracker/TUNING_GUIDE.md에 완전한 튜닝 가이드를 작성했습니다.

  현재 문제: 조향 반응이 느림

  즉시 적용 가능한 해결책 (노드 재시작 불필요):

  # 1. Stanley controller gain 증가 (횡방향 오차 수정 속도 향상)
  ros2 param set /path_tracker_node stanley_k 1.2

  # 2. 스티어링 필터 반응성 증가 (필터링 감소)
  ros2 param set /path_tracker_node steering_alpha 0.8

  # 3. k_error 부호 확인 및 수정
  ros2 param get /path_tracker_node k_error
  # 만약 양수(0.3)라면 음수로 변경:
  ros2 param set /path_tracker_node k_error -0.3

  # 4. (선택사항) lookahead 거리 감소로 더 빠른 반응
  ros2 param set /path_tracker_node lookahead_base 0.25

  주요 파라미터 설명

  빠른 반응을 위해 조정할 파라미터:

  1. stanley_k (현재: 0.5 → 권장: 1.0~1.5)
    - 횡방향 오차 수정 게인
    - 높을수록 빠른 수정, 낮을수록 부드러움
  2. steering_alpha (현재: 0.5 → 권장: 0.7~0.8)
    - 스티어링 필터 계수 (0=매우 부드러움, 1=필터 없음)
    - 높을수록 반응적, 낮을수록 부드러움
  3. k_error (현재: 0.3 → 수정 필요: -0.3)
    - 중요: 양수면 잘못된 설정!
    - 음수여야 큰 오차 발생 시 lookahead를 줄여 빠른 수정
  4. lookahead_base (현재: 0.3m → 권장: 0.25m)
    - 낮을수록 빠른 반응, 높을수록 안정적

  테스트 절차

  1. 먼저 위의 명령어로 즉시 테스트 (재시작 불필요)
  2. 결과 관찰:
    - 개선됨 → 설정 유지
    - 진동/오버슛 → stanley_k를 0.8~1.0으로 낮추기
    - 여전히 느림 → stanley_k를 1.5까지 올리기
  3. 만족스러우면 config 파일에 영구 저장:
  nano ~/f1tenth_dawgs/src/controller/path_tracker/config/tracker_params.yaml

  추가 설정 옵션

  TUNING_GUIDE.md에서 제공하는 추천 설정:

  최대 반응성 (실험적):
  lookahead_base: 0.2
  stanley_k: 1.5
  steering_alpha: 0.9
  k_error: -0.5

  공격적 레이싱 (균형잡힌 빠른 반응):
  lookahead_base: 0.25
  stanley_k: 1.2
  steering_alpha: 0.8
  k_error: -0.4

  가이드에 모든 파라미터의 의미, 효과, 튜닝 방법이 상세히 설명되어 있습니다!

--- tuning guide v2
● Path tracker의 조향 구조와 문제점을 분석했습니다.

  📊 Path Tracker 조향 구조

  1️⃣ Pure Pursuit 기반 조향 (path_tracker_node.cpp:497-520)

  Lookahead point → Vehicle frame 변환 → Pure pursuit 공식
  steering = atan2(2 * wheelbase * sin(α), L)
  - α: 차량에서 lookahead point까지 각도
  - L: lookahead 거리

  2️⃣ Stanley Controller 보정 추가 (line:1019-1044)

  Total correction = stanley_term + heading_error
  - stanley_term = atan(stanley_k * lateral_error / velocity)
  - heading_error = path_yaw - vehicle_yaw

  3️⃣ 최종 조향 계산 (line:356-370)

  Combined steering = Pure Pursuit + Stanley correction
  → Low-pass filter 적용
  → Max steering angle clamp

  ⚠️ 현재 문제점 진단

  🔴 Critical Issue #1: 너무 짧은 Lookahead

  lookahead_base: 0.3m        # 기본값이 너무 짧음
  lookahead_min: 0.4m         # 최소값도 짧음
  k_error: -0.5               # ❌ 역효과: error 클수록 lookahead 감소
  문제: Lateral error 발생 → lookahead 감소 → 더 급격한 조향 → 오버슈팅 →
  악순환

  🟡 Issue #2: Stanley Gain 과민성

  stanley_k: 1.2              # 높은 보정 gain
  use_stanley: true           # Heading error도 추가
  문제: Pure pursuit + Stanley 결합 → 과도한 조향 명령 → Oscillation

  🟡 Issue #3: 낮은 필터링 효과

  steering_alpha: 0.9         # 90% 새 값, 10% 이전 값
  문제: 필터링이 거의 없어서 급격한 조향 변화 허용 → 차량 불안정

  🟢 Issue #4: Adaptive Lookahead 역효과

  // line:1001 - Error 증가 시 lookahead 감소 (잘못된 로직)
  error_adjustment = -k_error_ * std::abs(lateral_error);
  adaptive_lookahead = lookahead_min_ + curv_adjustment + error_adjustment;
  올바른 동작: Error 클 때 → lookahead 증가 → 부드러운 복귀현재 동작: Error
  클 때 → lookahead 감소 → 급격한 조향 → 더 큰 error

  💡 권장 수정사항

  Priority 1: Lookahead 증가

  lookahead_base: 1.0  # 0.3 → 1.0
  lookahead_min: 0.8   # 0.4 → 0.8
  k_error: 0.3         # -0.5 → 0.3 (부호 반전, error 클수록 lookahead 증가)

  Priority 2: Stanley Gain 감소

  stanley_k: 0.3       # 1.2 → 0.3 (75% 감소)
  # 또는 use_stanley: false  (일시적으로 비활성화 테스트)

  Priority 3: 필터링 강화

  steering_alpha: 0.3  # 0.9 → 0.3 (부드러운 조향)

  Priority 4: Curvature Gain 조정

  k_curvature: 1.0     # 0.5 → 1.0 (곡률에 더 민감하게 반응)

  🔍 추가 검증 사항

  1. Lookahead 시각화 확인: /lookahead_point 토픽에서 lookahead point가 너무
   가까운지 RViz로 확인
  2. Lateral error 모니터링: DEBUG 로그에서 lat_err 값이 oscillation 하는지
  확인
  3. Steering 변화율: 조향각이 급격하게 변하는지 /drive 토픽 모니터링

  수정 후에도 문제가 지속되면 말씀해주세요!
