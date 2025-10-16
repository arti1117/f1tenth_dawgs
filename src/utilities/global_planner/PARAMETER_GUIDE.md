# Global Planner Parameter Configuration Guide

## Overview

Global planner는 TUM의 최소 시간 궤적 최적화 알고리즘을 사용합니다. 이 가이드는 차량별 파라미터 조정 방법을 설명합니다.

## 파라미터 파일 구조

파라미터는 `.ini` 파일로 관리됩니다:
- `racecar.ini`: 기본 포뮬러 E 차량 (참고용)
- `racecar_f110.ini`: F1TENTH 차량 (현재 사용 중)

위치: `src/utilities/global_planner/global_planner/global_racetrajectory_optimization/global_racetrajectory_optimization/params/`

---

## 🚗 1. 일반 차량 파라미터 (GENERAL_OPTIONS)

### veh_params - 기본 차량 특성

```ini
veh_params = {
    "v_max": 15.0,      # [m/s] 최고 속도 (현재: 54 km/h)
    "length": 0.535,    # [m] 차량 길이
    "width": 0.30,      # [m] 차량 폭
    "mass": 3.518,      # [kg] 차량 무게
    "dragcoeff": 0.0136,# [kg*m²/m³] 공기저항 = 0.5 * ρ_air * c_w * A_front
    "curvlim": 1.0,     # [rad/m] 최대 곡률 (최소 회전 반경 = 1m)
    "g": 9.81           # [m/s²] 중력 가속도
}
```

**조정 방법:**
- **v_max**: 차량의 실제 최고 속도 측정 후 입력
- **mass**: 차량 무게 측정 (배터리 포함)
- **dragcoeff**: 측정 어려움, 비슷한 크기 차량 참고
  - F1TENTH: 0.0136 (낮은 공기저항)
  - 일반 RC카: 0.02-0.05
- **curvlim**: 최소 회전 반경의 역수
  - 측정: 최대 조향각으로 회전 → 반경 측정 → 1/반경

### stepsize_opts - 궤적 이산화 옵션

```ini
stepsize_opts = {
    "stepsize_prep": 0.05,              # [m] 전처리 보간 간격
    "stepsize_reg": 0.2,                # [m] 최적화 중 간격 (노말 벡터 수)
    "stepsize_interp_after_opt": 0.1    # [m] 최적화 후 보간 간격
}
```

**조정 가이드:**
- 작은 값 = 더 정밀한 궤적, 더 느린 계산
- **stepsize_reg** 조정 권장:
  - 좁은 트랙 (< 3m): 0.1-0.2m
  - 넓은 트랙 (> 5m): 0.3-0.5m

---

## 🏎️ 2. 최적화 옵션 (OPTIMIZATION_OPTIONS)

### optim_opts_mintime - 최소 시간 최적화

```ini
optim_opts_mintime = {
    "width_opt": 0.8,       # [m] 안전 거리 포함 차량 폭
    "penalty_delta": 1.0,   # [-] 조향 평활도 페널티 (0-50)
    "penalty_F": 0.1,       # [-] 힘 평활도 페널티 (0-2)
    "mue": 1.0,             # [-] 마찰 계수 (타이어-노면)
    ...
}
```

**중요 파라미터:**
- **width_opt**: 실제 차량 폭 + 안전 여유 (0.1-0.2m 추가 권장)
- **mue**: 노면 마찰 계수
  - 실내 매끄러운 바닥: 0.6-0.8
  - 실내 콘크리트: 0.8-1.0
  - 아스팔트: 1.0-1.2
  - 고무 트랙: 1.2-1.5
- **penalty_delta/penalty_F**: 더 부드러운 궤적 원하면 증가

---

## 🔧 3. 차량 동역학 파라미터 (vehicle_params_mintime)

```ini
vehicle_params_mintime = {
    "wheelbase_front": 0.15875,  # [m] 전륜 휠베이스
    "wheelbase_rear": 0.17145,   # [m] 후륜 휠베이스
    "track_width_front": 0.281,  # [m] 전륜 트랙 폭
    "track_width_rear": 0.281,   # [m] 후륜 트랙 폭
    "cog_z": 0.074,              # [m] 무게 중심 높이
    "I_z": 0.04712,              # [kg*m²] 요 관성 모멘트
    "k_brake_front": 0.5,        # [-] 전륜 제동력 분배 (0-1)
    "k_drive_front": 0.0,        # [-] 전륜 구동력 분배 (0=RWD)
    "k_roll": 0.5,               # [-] 전륜 롤 모멘트 분배
    "delta_max": 0.34,           # [rad] 최대 조향각 (≈19.5°)
    "power_max": 267,            # [W] 최대 출력
    "f_drive_max": 33.4,         # [N] 최대 구동력
    "f_brake_max": 47.4          # [N] 최대 제동력
}
```

### 측정 방법:

**wheelbase (휠베이스):**
- 전륜: 무게 중심에서 전륜 축까지 거리
- 후륜: 무게 중심에서 후륜 축까지 거리
- 합: 전륜 축-후륜 축 거리

**cog_z (무게 중심 높이):**
1. 차량을 평평한 곳에 놓고 한쪽 끝을 들어올림
2. 들어올린 높이와 기울기 측정
3. 기하학적 계산 또는 추정 (대략 지면에서 차량 높이의 40-50%)

**I_z (요 관성 모멘트):**
- 정확한 측정 어려움
- 근사식: `I_z ≈ mass * (length² + width²) / 12`
- F1TENTH 예: `3.518 * (0.535² + 0.30²) / 12 ≈ 0.106`
- 실제는 더 작음 (배터리 중앙 집중)

**delta_max (최대 조향각):**
- 서보 모터 최대 각도 측정 (기계적 한계)
- VESC 설정의 `servo_max_angle` 확인

**f_drive_max/f_brake_max (최대 힘):**
- 측정: 차량에 로드셀 부착하여 최대 가속/제동 시 측정
- 추정: `F = mass * a_max`
  - 가속: `3.518 kg * 9.5 m/s² ≈ 33.4 N`
  - 제동: `3.518 kg * 13.5 m/s² ≈ 47.4 N`

---

## 🛞 4. 타이어 파라미터 (tire_params_mintime)

### Magic Formula 타이어 모델

```ini
tire_params_mintime = {
    "c_roll": 0.010,        # [-] 구름 저항 계수
    "f_z0": 8.6,            # [N] 공칭 수직력 (바퀴당)
    "B_front": 7.4,         # [-] Magic Formula B 계수 (전륜)
    "C_front": 1.2,         # [-] Magic Formula C 계수
    "eps_front": -0.1,      # [-] 수직력 의존성 계수
    "E_front": 0.85,        # [-] Magic Formula E 계수
    "B_rear": 7.4,          # [-] 후륜 계수
    "C_rear": 1.2,
    "eps_rear": -0.1,
    "E_rear": 0.85
}
```

### Magic Formula 이론

타이어 측력/종력 모델: **Pacejka Magic Formula**

```
F_y = D * sin(C * arctan(B * α - E * (B * α - arctan(B * α))))
```

여기서:
- **F_y**: 타이어 측력 [N]
- **α**: 슬립각 [rad]
- **D**: Peak factor = `F_z * mue * (1 + eps_front * (F_z - f_z0) / f_z0)`
- **B**: Stiffness factor (커질수록 가파른 기울기)
- **C**: Shape factor (피크 형상, 보통 1.0-2.5)
- **E**: Curvature factor (피크 후 하강 곡선)

### 다른 차량에 타이어 모델 적용하기

#### 방법 1: 유사 차량 참고 (추천)

**F1TENTH 스케일 차량 (1/10 RC)**
```ini
# 현재 F1TENTH 설정
f_z0 = 8.6  # N (= 3.518 kg * 9.81 / 4 바퀴)
B_front = 7.4
C_front = 1.2
E_front = 0.85
```

**더 무거운 차량 (예: 5kg)**
```ini
f_z0 = 12.3  # N (= 5.0 kg * 9.81 / 4)
B_front = 6.5  # 무거우면 약간 감소
C_front = 1.2
E_front = 0.85
```

**더 가벼운 차량 (예: 2kg)**
```ini
f_z0 = 4.9  # N (= 2.0 kg * 9.81 / 4)
B_front = 8.5  # 가벼우면 약간 증가
C_front = 1.2
E_front = 0.85
```

#### 방법 2: 실험적 튜닝

1. **초기값 설정**
   ```python
   f_z0 = vehicle_mass * 9.81 / 4  # 4 바퀴
   B = 7.0  # 시작값
   C = 1.2
   E = 0.85
   ```

2. **주행 테스트**
   - 차량이 **언더스티어** (회전 부족) → B 증가 (8-10)
   - 차량이 **오버스티어** (회전 과다) → B 감소 (5-7)
   - 차량이 불안정 → E 증가 (0.9-1.0)

3. **반복 조정**
   - 시뮬레이션 궤적과 실제 주행 비교
   - 코너 속도가 실제보다 높으면 → B 감소 또는 mue 감소
   - 코너 속도가 실제보다 낮으면 → B 증가 또는 mue 증가

#### 방법 3: 타이어 데이터 시트 활용 (고급)

타이어 제조사 데이터가 있는 경우:
1. **Cornering Stiffness** (C_α) 확인
2. Magic Formula 변환:
   ```
   B ≈ C_α / (C * D)
   ```

#### 타이어 타입별 참고값

| 타이어 타입 | B (Stiffness) | C (Shape) | E (Curvature) | 용도 |
|------------|---------------|-----------|---------------|------|
| 고무 슬릭 (Soft) | 8-12 | 1.3-1.6 | 0.8-0.9 | 높은 그립, 경주용 |
| 고무 슬릭 (Hard) | 6-9 | 1.2-1.4 | 0.85-0.95 | 내구성, 연습용 |
| 플라스틱/ABS | 4-6 | 1.0-1.2 | 0.9-1.0 | 낮은 그립, 드리프트 |
| 폼 타이어 | 5-7 | 1.1-1.3 | 0.85-0.9 | 실내 카펫 |

### c_roll (구름 저항) 조정

```ini
c_roll: 구름 저항 계수
- 슬릭 타이어: 0.008-0.012
- 범용 타이어: 0.012-0.018
- 오프로드: 0.020-0.030
```

---

## 📊 5. 실전 튜닝 워크플로우

### Step 1: 차량 계측
```bash
# 측정 항목 체크리스트
[ ] 차량 무게 (mass)
[ ] 차량 크기 (length, width)
[ ] 휠베이스 (wheelbase_front + wheelbase_rear)
[ ] 트랙 폭 (track_width)
[ ] 최대 조향각 (delta_max)
[ ] 최고 속도 (v_max)
```

### Step 2: 파라미터 파일 생성
```bash
# 새 차량용 파일 생성
cd src/utilities/global_planner/.../params/
cp racecar_f110.ini my_vehicle.ini
```

### Step 3: 기본 파라미터 입력
```ini
# my_vehicle.ini 편집
[GENERAL_OPTIONS]
veh_params = {
    "mass": <측정값>,
    "length": <측정값>,
    "width": <측정값>,
    "v_max": <측정값>,
    ...
}
```

### Step 4: 타이어 파라미터 추정
```python
# Python 계산 스크립트
mass = 3.518  # kg
f_z0 = mass * 9.81 / 4  # 바퀴당 수직력

# 유사 차량 참고 시작값
B = 7.4
C = 1.2
E = 0.85
```

### Step 5: 최적화 실행 및 검증
```bash
# Global planner 실행
python3 main_globaltraj.py

# 생성된 궤적 확인
# outputs/traj_race_cl.csv
```

### Step 6: 실차 검증 및 튜닝
```python
# 실제 주행과 비교
if lap_time_sim < lap_time_real:
    # 시뮬이 더 빠름 → 타이어 그립 과대평가
    B *= 0.9  # 10% 감소
elif lap_time_sim > lap_time_real:
    # 시뮬이 더 느림 → 타이어 그립 과소평가
    B *= 1.1  # 10% 증가
```

---

## 🔍 6. 주요 파라미터 영향 분석

### 마찰 계수 (mue)
- **영향**: 코너 최대 속도, 전체 랩타임
- **0.8 → 1.0**: 랩타임 약 5-10% 감소
- **측정**: 최대 횡가속도 측정 → `mue ≈ a_y_max / g`

### Stiffness (B)
- **영향**: 타이어 반응성, 저속 코너 성능
- **높음 (10)**: 선형적 반응, 예측 가능
- **낮음 (5)**: 비선형적, 슬립 각도 증가

### Shape (C)
- **영향**: 피크 형상
- **C < 1.3**: 날카로운 피크 (경주용)
- **C > 1.3**: 넓은 피크 (안정적)

### Curvature (E)
- **영향**: 한계 영역 거동
- **E < 0.9**: 부드러운 한계 (안전)
- **E > 0.9**: 급격한 그립 손실 (날카로움)

---

## 📖 7. 참고 자료

### 학술 논문
1. Pacejka, H. B. (2012). *Tire and Vehicle Dynamics* (3rd ed.)
2. Heilmeier et al. (2020). "Minimum Curvature Trajectory Planning"

### 온라인 리소스
- TUM Global Planner: https://github.com/TUMFTM/global_racetrajectory_optimization
- F1TENTH Docs: https://f1tenth.org

### Magic Formula 시뮬레이터
```python
import numpy as np
import matplotlib.pyplot as plt

def magic_formula(alpha, B, C, D, E):
    """Pacejka Magic Formula"""
    return D * np.sin(C * np.arctan(B * alpha - E * (B * alpha - np.arctan(B * alpha))))

# 파라미터 시각화
alpha = np.linspace(-0.3, 0.3, 100)
F_y = magic_formula(alpha, B=7.4, C=1.2, D=8.6, E=0.85)

plt.plot(np.degrees(alpha), F_y)
plt.xlabel('Slip Angle [deg]')
plt.ylabel('Lateral Force [N]')
plt.grid(True)
plt.show()
```

---

## ⚠️ 주의사항

1. **안전 우선**: 처음에는 보수적인 값 사용 (낮은 mue, 작은 B)
2. **점진적 튜닝**: 한 번에 하나의 파라미터만 변경
3. **실차 검증**: 시뮬레이션 결과를 항상 실차로 검증
4. **백업**: 작동하는 파라미터 파일 백업 유지

---

## 🚀 빠른 시작 (Quick Start)

### 새 F1TENTH 차량 설정 (3분)

```bash
# 1. 차량 무게 측정
mass = 3.8  # kg (예시)

# 2. 파라미터 파일 복사
cp racecar_f110.ini my_f110.ini

# 3. 필수 값만 수정
nano my_f110.ini
# - veh_params["mass"] = 3.8
# - tire_params_mintime["f_z0"] = 3.8 * 9.81 / 4 = 9.32

# 4. main_globaltraj.py에서 파일명 변경
file_paths["veh_params_file"] = "my_f110.ini"

# 5. 실행
python3 main_globaltraj.py
```

### 트러블슈팅

**문제: 최적화가 수렴하지 않음**
- `width_opt` 증가 (0.8 → 1.0)
- `stepsize_reg` 증가 (0.2 → 0.3)

**문제: 궤적이 트랙을 벗어남**
- `width_opt` 감소
- `min_track_width` 설정 (main_globaltraj.py)

**문제: 궤적이 너무 공격적**
- `mue` 감소 (1.0 → 0.8)
- `safe_traj = true` 설정
- `ax_pos_safe`, `ay_safe` 제한 추가

---

작성: 2025
버전: 1.0
