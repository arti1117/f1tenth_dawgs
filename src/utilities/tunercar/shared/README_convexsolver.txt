네, 연구 논문들을 참조해서 tunercar용 convex_solver를 성공적으로 구성했습니다.

  주요 구현 내용:

  1. Minimum-Time Speed Optimization (Lipp & Boyd 2014 기반)

  - Path reparameterization: θ 매개변수를 사용한 경로 재정의
  - Convex transformation: b(θ) = θ̇² 변환을 통한 볼록화
  - High-order finite differences: 6차 대칭 스텐실을 사용한 경로 미분
  - Vehicle dynamics constraints: 가속도, 곡률, 마찰 제약 조건

  2. TUNERCAR 통합

  - Multi-level optimization: 속도 최적화 + 매개변수 튜닝
  - Trust region methods: 비볼록 매개변수 최적화
  - Vehicle parameter identification: F1TENTH 차량 매개변수 적용

  3. 핵심 기능

  - MinimumTimeSpeedOptimizer: 고정 경로에 대한 최적 속도 프로파일
  - ConvexSolver: 일반적인 볼록 최적화 해결사
  - solve_tunercar_convex: TUNERCAR 통합 래퍼 함수
  - optimize_racing_line_and_speed: 전체 레이싱 라인 + 속도 최적화

  4. 구현된 파일들

  - /shared/convex_solver.py: 핵심 볼록 최적화 모듈
  - /shared/pipeline_convex.py: 업데이트된 파이프라인
