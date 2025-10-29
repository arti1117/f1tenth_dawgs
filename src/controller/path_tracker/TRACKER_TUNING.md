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

--- speed tuning guide
● path_tracker 패키지의 speed_mode에 대한 각 모드의 동작 방식과 제약사항을
  설명드리겠습니다.

  Speed Mode 종류 및 동작 방식

  1. "default" 모드

  동작 방식:
  - 고정된 default_speed 값을 사용합니다
  - 경로의 기하학적 정보나 차량 상태를 고려하지 않습니다

  제약사항:
  - ✅ 가장 단순하고 예측 가능한 동작
  - ⚠️ 경로 정보를 무시하므로 곡선 구간에서 과속 위험
  - ⚠️ 직선 구간에서도 느린 속도 유지로 비효율적
  - 📍 적용 위치: path_tracker_node.cpp:844

  ---
  2. "path_velocity" 모드

  동작 방식:
  1. 1차 우선순위: lookahead.v (frenet_path에 내장된 속도) 사용
    - pose.position.z > 0.01인 경우 해당 값 사용
  2. Fallback: global_centerline의 속도 프로파일 사용
    - lookahead 포인트와 가장 가까운 global path 지점의 속도 참조

  제약사항:
  - ✅ 사전 최적화된 속도 프로파일 활용 (시간 최적화 경로에 최적)
  - ⚠️ global_path_topic 구독 필요 (/global_centerline)
  - ⚠️ global path가 없으면 default_speed로 fallback
  - ⚠️ path_planner가 속도 정보를 제공하지 않으면 부정확할 수 있음
  - 📍 적용 위치: path_tracker_node.cpp:793-812

  코드 흐름:
  if (lookahead.v > 0.01) {
      speed = lookahead.v;  // frenet path 속도 사용
  } else {
      speed = getGlobalPathSpeed(lookahead.x, lookahead.y);  // global 
  path에서 검색
  }

  ---
  3. "curvature" 모드

  동작 방식:
  - 경로의 곡률(curvature)을 기반으로 최대 안전 속도를 계산
  - 물리 공식: v_max = sqrt(μ * g / |κ|)
    - μ (friction_coeff): 마찰 계수 (기본 0.9)
    - g: 중력 가속도 (9.81 m/s²)
    - κ (kappa): 경로 곡률

  제약사항:
  - ✅ 경로 기하학에 자동 적응 (곡선에서 감속, 직선에서 가속)
  - ✅ 물리적으로 안전한 속도 계산
  - ⚠️ 곡률이 매우 작으면 (|κ| < 1e-6) max_speed_limit 사용
  - ⚠️ friction_coeff 값이 부정확하면 과속/저속 문제 발생
  - ⚠️ 3-point finite difference로 곡률 계산 → 최소 3개 waypoint 필요
  - 📍 적용 위치: path_tracker_node.cpp:815-818, 896-926

  곡률 계산 방식 (path_tracker_node.cpp:928-952):
  // 3점(p1, p2, p3)을 사용한 곡률 계산
  κ = 2 * |cross(v1, v2)| / |v1|³

  ---
  4. "optimize" 모드

  동작 방식:
  - 3가지 속도의 최소값(min) 선택:
    a. Path velocity (frenet path 또는 global path)
    b. Curvature-based speed
    c. (Steering limit은 나중에 별도 적용)

  제약사항:
  - ✅ 가장 보수적이고 안전한 모드
  - ✅ 모든 제약 조건 동시 고려
  - ⚠️ global_path_topic 구독 필요
  - ⚠️ 가장 느린 속도로 동작할 수 있음 (안전 vs 성능 trade-off)
  - ⚠️ Path velocity와 curvature 모드의 모든 제약사항 상속
  - 📍 적용 위치: path_tracker_node.cpp:820-838

  선택 로직:
  speed = min(path_velocity, curvature_speed);
  // Steering limit는 applyAccelerationLimit에서 별도 적용

  ---
  공통 제약사항 (모든 모드)

  속도 제한

  1. 속도 클램핑 (path_tracker_node.cpp:849):
  speed = clamp(speed, min_speed_limit, max_speed_limit)
    - min_speed_limit: 0.5 m/s (기본값)
    - max_speed_limit: 8.0 m/s (기본값)
  2. Debug 모드 적용 (path_tracker_node.cpp:852-865):
  if (debug_mode) {
      speed *= velocity_gain;  // 기본 0.3 (30% 속도)
      speed = max(speed, debug_min_speed);  // 최소 0.6 m/s
  }
  3. Friction Circle 제약 (use_acceleration_limit=true):
    - 횡방향 가속도를 고려한 종방향 가속도 제한
    - a_total² = a_lateral² + a_longitudinal²
    - 📍 path_tracker_node.cpp:562-575, 1164-1194

  ---
  모드별 추천 시나리오

  | 모드            | 추천 시나리오             | 주의사항                 |
  |---------------|---------------------|----------------------|
  | default       | 테스트, 간단한 경로 추종      | 안전 속도로 설정 필수
        |
  | path_velocity | 시간 최적화된 레이싱         | Global path 필수       |
  | curvature     | 실시간 적응형 주행, SLAM 기반 | friction_coeff 튜닝 필요
   |
  | optimize      | 최대 안전성 요구 환경        | 느릴 수 있음
   |

  ---
  현재 설정 확인

  현재 tracker_params.yaml:202에서 "curvature" 모드로 설정되어 있으며:
  - friction_coeff: 0.9
  - max_speed_limit: 8.0 m/s
  - min_speed_limit: 0.5 m/s
  - debug_mode: true (velocity_gain: 0.3 → 실제 속도는 계산값의 30%)

  각 모드에 대해 더 자세한 설명이나 특정 상황에서의 튜닝 가이드가 필요하시면
   말씀해 주세요!

--- weighted optimize mode
● 완료되었습니다! ✅ optimize 모드에 가중치 기반 계산 기능을 추가했습니다.

  변경사항 요약

  1. 파라미터 추가 (tracker_params.yaml:240-262)

  # OPTIMIZE MODE PARAMETERS
  optimize_use_weighted: false    # true = 가중치 사용, false = min() 사용
  optimize_path_weight: 0.5        # path_velocity 가중치 (0.0~1.0)
  optimize_curv_weight: 0.5        # curvature 가중치 (0.0~1.0)

  2. 계산 로직 (path_tracker_node.cpp:830-872)

  기존 방식 (optimize_use_weighted: false)

  speed = min(path_velocity, curvature_speed)

  새로운 가중치 방식 (optimize_use_weighted: true)

  // 가중치 자동 정규화 (합이 1.0이 되도록)
  norm_path_weight = path_weight / (path_weight + curv_weight)
  norm_curv_weight = curv_weight / (path_weight + curv_weight)

  speed = norm_path_weight × path_velocity + norm_curv_weight ×
  curvature_speed

  사용 방법

  1. 가중치 모드 활성화

  tracker_params.yaml 수정:
  speed_mode: "optimize"
  optimize_use_weighted: true
  optimize_path_weight: 0.7   # 경로 속도 70%
  optimize_curv_weight: 0.3   # 곡률 속도 30%

  2. 빌드 및 실행

  source install/setup.bash
  ros2 launch path_tracker path_tracker.launch.py

  3. 튜닝 가이드

  | 시나리오    | path_weight | curv_weight | 효과                        |
  |---------|-------------|-------------|---------------------------|
  | 공격적 레이싱 | 0.8         | 0.2         | 최적화된 경로 속도 우선
          |
  | 균형잡힌 주행 | 0.5         | 0.5         | 경로와 기하학 동등 고려
          |
  | 안전 우선   | 0.3         | 0.7         | 기하학적 안전 속도 우선
        |
  | 매우 보수적  | 0.0         | 1.0         | 곡률만 고려 (curvature 모드와
   동일) |

  4. 디버그 로그 확인

  ros2 topic echo /rosout | grep OPTIMIZE

  출력 예시:
  OPTIMIZE [WEIGHTED]: path=6.50 (w=0.70), curv=5.00 (w=0.30) → 6.15 m/s

  주요 기능

  ✅ 안전 기능

  - 가중치 자동 정규화: 합이 1.0이 되도록 자동 조정
  - Zero 가중치 보호: 양쪽 모두 0이면 자동으로 (0.5, 0.5) 적용
  - 하위 호환성: optimize_use_weighted: false로 기존 min() 방식 유지

  📊 장점

  - 부드러운 속도 프로파일: min()의 급격한 변화 방지
  - 튜닝 가능: 트랙/상황에 맞게 가중치 조정 가능
  - 디버그 친화적: 각 속도 성분과 가중치를 로그로 확인

  ⚠️ 주의사항

  - path_velocity 모드처럼 global_path_topic 구독 필요
  - 가중치 조합이므로 min()보다 빠른 속도가 나올 수 있음
  - 트랙 테스트를 통해 최적 가중치 찾기 권장

  예제 설정

  레이싱 모드 (공격적)

  speed_mode: "optimize"
  optimize_use_weighted: true
  optimize_path_weight: 0.8
  optimize_curv_weight: 0.2
  friction_coeff: 1.0
  max_speed_limit: 9.0

  안전 모드 (보수적)

  speed_mode: "optimize"
  optimize_use_weighted: true
  optimize_path_weight: 0.3
  optimize_curv_weight: 0.7
  friction_coeff: 0.8
  max_speed_limit: 7.0

  변경사항이 정상적으로 빌드되었으며, 바로 사용하실 수 있습니다! 🚀

