# Global Planner 파라미터 설정 가이드

## 빠른 시작 (Quick Start)

### 1. 인터랙티브 설정 도구 사용 (추천)

```bash
cd src/utilities/global_planner/global_planner/global_racetrajectory_optimization/global_racetrajectory_optimization
python3 vehicle_config_generator.py
```

**실행 예시:**
```
🏎️  Global Planner Vehicle Configuration Generator
======================================================================

📏 STEP 1: Basic Vehicle Measurements
----------------------------------------------------------------------
Vehicle mass [kg] (e.g., 3.518): 3.5
Vehicle length [m] (e.g., 0.535): 0.53
Vehicle width [m] (e.g., 0.30): 0.28
Maximum speed [m/s] (e.g., 15.0): 12.0

✅ Calculated defaults:
  - Vertical force per tire: 8.60 N
  - Yaw inertia: 0.04105 kg·m²
  - Drag coefficient: 0.0129

🛞 STEP 2: Tire Configuration
----------------------------------------------------------------------
Available tire presets:
  1. Soft Racing Slicks (high grip, racing)
  2. Hard Racing Slicks (durable, practice)
  3. Foam Tires (indoor carpet)
  4. Plastic/ABS (low grip, drifting)
  5. Drift Spec (controllable oversteer)
  6. Custom (manual input)

Select tire preset [1-6] (default: 2): 2

✅ Selected: Durable racing slicks (hard compound)
  - Stiffness (B): 7.4
  - Shape (C): 1.2
  - Curvature (E): 0.9

🏁 STEP 3: Track Surface
----------------------------------------------------------------------
Surface friction coefficient (μ):
  1. Indoor smooth floor (0.7)
  2. Indoor concrete (0.9)
  3. Asphalt (1.0)
  4. Rubber track (1.2)
  5. Custom

Select surface [1-5] (default: 3): 2
✅ Selected friction: μ = 0.9

⚙️  STEP 4: Drivetrain Configuration
----------------------------------------------------------------------
Drivetrain type:
  1. Rear-Wheel Drive (RWD)
  2. Front-Wheel Drive (FWD)
  3. All-Wheel Drive (AWD)

Select drivetrain [1-3] (default: 1): 1

📐 STEP 5: Vehicle Geometry (Optional)
----------------------------------------------------------------------
[Press Enter to use estimated values]

💾 STEP 6: Save Configuration
----------------------------------------------------------------------
Configuration filename (without .ini, e.g., 'my_vehicle'): my_f110

✅ Configuration saved to: my_f110.ini

📝 Summary:
  Mass: 3.5 kg
  Size: 0.53 x 0.28 m
  Max speed: 12.0 m/s (43.2 km/h)
  Friction: μ = 0.9
  Tire: B=7.4, C=1.2, E=0.9
  Drivetrain: RWD

🚀 Next steps:
  1. Edit main_globaltraj.py:
     file_paths['veh_params_file'] = 'my_f110.ini'
  2. Run: python3 main_globaltraj.py
  3. Test generated trajectory on vehicle
  4. Tune parameters based on real performance
```

### 2. 생성된 파라미터 파일 사용

```python
# main_globaltraj.py 파일 수정
file_paths = {"veh_params_file": "my_f110.ini"}  # 여기를 변경!

# 나머지 설정...
opt_type = 'mintime'
```

### 3. 최적화 실행

```bash
cd src/utilities/global_planner/global_planner/global_racetrajectory_optimization
python3 main_globaltraj.py
```

---

## 📚 상세 문서

전체 파라미터 설명과 타이어 모델 이론은 다음 문서를 참조하세요:
- **[PARAMETER_GUIDE.md](PARAMETER_GUIDE.md)** - 완전한 파라미터 가이드 및 Magic Formula 설명

---

## 🔧 수동 설정 (고급 사용자)

기존 `.ini` 파일을 복사하여 직접 편집할 수도 있습니다:

```bash
cd params/
cp racecar_f110.ini my_custom_vehicle.ini
nano my_custom_vehicle.ini
```

### 필수 수정 항목

```ini
[GENERAL_OPTIONS]
veh_params = {
    "mass": 3.5,          # 실제 차량 무게
    "length": 0.53,       # 차량 길이
    "width": 0.28,        # 차량 폭
    "v_max": 12.0,        # 최고 속도
    ...
}

[OPTIMIZATION_OPTIONS]
optim_opts_mintime = {
    "mue": 0.9,           # 노면 마찰 계수
    ...
}

tire_params_mintime = {
    "f_z0": 8.6,          # 바퀴당 수직력 = mass * 9.81 / 4
    "B_front": 7.4,       # 타이어 강성
    "C_front": 1.2,       # 타이어 형상
    "E_front": 0.85,      # 타이어 곡률
    ...
}
```

---

## 🎯 파라미터 튜닝 가이드

### 시뮬레이션이 실제보다 빠른 경우

**원인**: 타이어 그립이나 마찰이 과대평가됨

**해결책**:
```ini
# 방법 1: 마찰 계수 감소
optim_opts_mintime = {"mue": 0.8}  # 1.0 → 0.8

# 방법 2: 타이어 강성 감소
tire_params_mintime = {"B_front": 6.5}  # 7.4 → 6.5
```

### 시뮬레이션이 실제보다 느린 경우

**원인**: 타이어 그립이나 마찰이 과소평가됨

**해결책**:
```ini
# 방법 1: 마찰 계수 증가
optim_opts_mintime = {"mue": 1.1}  # 1.0 → 1.1

# 방법 2: 타이어 강성 증가
tire_params_mintime = {"B_front": 8.5}  # 7.4 → 8.5
```

### 차량이 언더스티어 (회전 부족)

**해결책**:
```ini
# 전륜 타이어 강성 증가
tire_params_mintime = {
    "B_front": 9.0,  # 증가
    "B_rear": 7.0    # 후륜은 유지 또는 감소
}
```

### 차량이 오버스티어 (회전 과다)

**해결책**:
```ini
# 후륜 타이어 강성 증가 또는 전륜 감소
tire_params_mintime = {
    "B_front": 6.5,  # 감소
    "B_rear": 8.5    # 증가
}
```

### 궤적이 너무 공격적

**해결책**:
```ini
# 안전 모드 활성화
optim_opts_mintime = {
    "safe_traj": true,
    "ax_pos_safe": 8.0,   # 최대 가속 제한 [m/s²]
    "ax_neg_safe": 10.0,  # 최대 감속 제한 [m/s²]
    "ay_safe": 8.0        # 최대 횡가속 제한 [m/s²]
}
```

---

## 📊 차량 측정 가이드

### 필수 측정값

| 항목 | 측정 방법 | 예시 값 (F1TENTH) |
|------|-----------|-------------------|
| **mass** | 저울로 측정 (배터리 포함) | 3.518 kg |
| **length** | 가장 앞-뒤 거리 | 0.535 m |
| **width** | 가장 넓은 부분 | 0.30 m |
| **v_max** | GPS 또는 휠 속도 측정 | 15.0 m/s |
| **wheelbase** | 전륜 축 - 후륜 축 거리 | 0.33 m |

### 권장 측정값 (더 정확한 결과)

| 항목 | 측정 방법 | 예시 값 |
|------|-----------|---------|
| **wheelbase_front** | 무게중심 - 전륜 축 | 0.159 m |
| **wheelbase_rear** | 무게중심 - 후륜 축 | 0.171 m |
| **track_width** | 좌우 바퀴 중심 거리 | 0.281 m |
| **cog_z** | 무게중심 높이 | 0.074 m |
| **delta_max** | 서보 최대 각도 | 0.34 rad (19.5°) |

### 마찰 계수 추정

실제 측정이 어려우므로, 다음 방법으로 추정:

**방법 1: 최대 횡가속도 측정**
```python
# 차량이 미끄러지기 직전의 최대 횡가속도 측정
a_y_max = 8.0  # m/s² (IMU로 측정)
mue = a_y_max / 9.81  # ≈ 0.82
```

**방법 2: 노면 타입 참고**
- 실내 매끄러운 바닥: 0.6-0.8
- 실내 콘크리트: 0.8-1.0
- 아스팔트: 1.0-1.2
- 고무 매트: 1.2-1.5

---

## 🧪 검증 체크리스트

설정 완료 후 다음을 확인하세요:

- [ ] 시뮬레이션이 에러 없이 완료됨
- [ ] 생성된 궤적이 트랙 경계 내에 있음
- [ ] 최대 속도가 `v_max` 이하임
- [ ] 실차 테스트에서 궤적 추종 가능
- [ ] 랩타임이 실제와 ±10% 이내

**문제 발생 시**:
1. 로그 메시지 확인
2. `PARAMETER_GUIDE.md` 트러블슈팅 섹션 참조
3. 파라미터를 보수적으로 조정 (낮은 마찰, 작은 B)

---

## 🚀 예제: 새 차량 설정 (5분 가이드)

### 시나리오: 새로 구입한 F1TENTH 차량

```bash
# 1. 차량 측정
# - 무게: 3.6 kg
# - 크기: 0.54 x 0.29 m
# - 최고 속도: 13 m/s (테스트 결과)
# - 타이어: 새 경주용 슬릭
# - 트랙: 실내 콘크리트

# 2. 설정 도구 실행
cd global_racetrajectory_optimization/global_racetrajectory_optimization
python3 vehicle_config_generator.py

# 3. 입력값:
Vehicle mass [kg]: 3.6
Vehicle length [m]: 0.54
Vehicle width [m]: 0.29
Maximum speed [m/s]: 13
Tire preset: 1 (soft slick)
Surface: 2 (indoor concrete, μ=0.9)
Drivetrain: 1 (RWD)
Filename: my_new_f110

# 4. main_globaltraj.py 수정
# file_paths["veh_params_file"] = "my_new_f110.ini"

# 5. 실행
python3 main_globaltraj.py

# 6. 실차 테스트 및 튜닝
# - 시뮬 랩타임: 25.3초
# - 실제 랩타임: 26.8초
# → 차이 6% → 양호! (±10% 이내)
```

---

## 📞 도움말

### 추가 리소스
- **전체 파라미터 가이드**: [PARAMETER_GUIDE.md](PARAMETER_GUIDE.md)
- **TUM GitHub**: https://github.com/TUMFTM/global_racetrajectory_optimization
- **F1TENTH Docs**: https://f1tenth.org

### 일반적인 질문

**Q: B, C, E 값을 어떻게 결정하나요?**
A: 타이어 프리셋을 사용하거나, 유사 차량 값을 참고하세요. 실차 테스트 후 튜닝하는 것이 가장 정확합니다.

**Q: 최적화가 너무 오래 걸립니다.**
A: `stepsize_reg`를 증가시키세요 (0.2 → 0.3). 정밀도는 약간 감소하지만 계산이 빨라집니다.

**Q: 차량이 궤적을 잘 따라가지 못합니다.**
A: 궤적이 너무 공격적일 수 있습니다. `safe_traj: true`로 설정하거나 `mue`를 감소시키세요.

---

**작성**: F1TENTH DAWGS Team, 2025
**버전**: 1.0
