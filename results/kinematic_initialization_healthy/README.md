# 운동학적 초기화 · 정상 타이어 우선 계산

**완료·검증: 2/3개.** 기존 210조건 일괄 계산은 초기화 변경으로 중단한 상태입니다.

각 MAT는 정상 타이어 [0 0 0 0]만 포함합니다. `STABILITY3D.CASE`는 하나이며 `meta.healthy_only=true`, `meta.case_count=1`입니다. 마모 5종 결과와 구분해 사용하세요. 초기조건 39,401개, 5초, Ts=0.01초, RK4 하위 적분 20회를 유지했습니다.

초기 횡슬립각은 초기 횡속도·요레이트·바퀴 조향각으로 계산합니다. 초기 휠 회전속도는 각 바퀴의 종방향 속도에 맞추고 초기 종방향 완화 슬립은 0입니다. 시뮬레이션 중 사용하는 바퀴 운동학 함수를 초기화에도 사용합니다. 이전 직접 초기화(-delta) 결과와는 초기조건 정의가 다릅니다.

| 조건 | MAT | 안정점 개수 |
|---|---|---|
| 80 km/h · μ=0.9 · 2° | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/kinematic_initialization_healthy/stable_point_80kph_0.9mu_2deg.mat) | 3062 |
| 50 km/h · μ=0.9 · 0° | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/kinematic_initialization_healthy/stable_point_50kph_0.9mu_0deg.mat) | 2847 |
| 50 km/h · μ=0.9 · 2° | 대기 또는 계산 중 | - |

[저장 데이터 검증](validation_summary.json) · [MATLAB/C++ 초기화 47개 검사](initialization_verification.json) · [단일 MATLAB 코드](../../matlab/stability_sweep_210.m)
