# 210조건 × 마모 5종 × 서스펜션 81모드 정상상태 맵

이 맵은 속도·노면 마찰계수·전륜 휠 조향각·타이어 마모·코너별 스프링 모드를 입력으로 하고 정상상태 횡속도, 요레이트, 롤각을 출력합니다. 총 조합 수는 **85,050개**입니다.

기존 210개 안정영역 MAT는 5초간 제약을 만족한 **초기상태**를 저장합니다. 여기의 정상상태 값은 같은 Full-Car/MF 동역학을 사용해 각 조합을 **새로 시뮬레이션**한 결과입니다. 기존 안정영역의 중심·평균을 수렴값으로 사용하지 않습니다.

## 축과 모드 순서

MAT 파일의 변수명은 `STEADY_STATE_LUT`입니다. 모든 주요 맵 배열의 크기는 `6 × 5 × 7 × 5 × 81`이며 인덱스 순서는 다음과 같습니다.

| 축 | 순서 |
|---|---|
| 속도, km/h | `50, 60, 70, 80, 90, 100` |
| 노면 μ | `0.5, 0.6, 0.7, 0.8, 0.9` |
| 전륜 휠 조향각, deg | `0, 0.5, 1, 1.5, 2, 2.5, 3` |
| 타이어 마모 | 아래의 마모 1~5 |
| 서스펜션 모드 | `nnnn, nnns, nnnh, nnsn, nnss, nnsh, …, hhhh` |

코너 순서는 항상 **FL, FR, RL, RR**입니다. 모드의 각 글자는 `n → s → h` 순서이며 **RR이 가장 빠르게 변합니다**. 전륜 휠 조향각은 `[δ, δ, 0, 0]`이고 종방향 속도는 주어진 km/h를 m/s로 변환한 값으로 고정합니다.

| 코너 | n, N/m | s, N/m | h, N/m |
|---|---:|---:|---:|
| FL, FR | 30790 | 22161 | 36086 |
| RL, RR | 30790 | 23632 | 35007 |

CDC 댐퍼 전류는 모든 코너·모드에서 **950 mA**입니다. 표의 값은 스프링 명령 강성이며 동역학 내부에서 기존 모션비의 제곱을 곱해 휠 레이트로 변환합니다.

타이어 마모 순서 `[FL FR RL RR]`:

```matlab
[ 0  0  0  0;   % 1: 정상 타이어
 25 25  0  0;   % 2
 50 50 25 25;   % 3
  0  0 25 25;   % 4
 25 25 50 50]   % 5
```

## 초기조건과 수렴 판정

각 조합의 초기 횡속도·요레이트·롤각은 0입니다. 기존 코드의 직접 시작 초기화, 즉 완화 슬립각 `-delta_w`, 자유 구름 휠 속도 `max(ux*cos(delta_w),0)/Rw`, 나머지 상태 0을 유지합니다. MF 타이어의 μ 배율도 기존과 같은 `mu/0.9`입니다.

RK4 적분은 `Ts=0.01 s`, 표본당 하위 적분 20회를 사용합니다. 최대 시뮬레이션 시간은 기본 60초이며 실제 적용값은 MAT의 `simulation_time_limit_s`와 `provenance.run_metadata_json`에 기록합니다. 수렴은 적어도 5초가 지난 시점부터 2초 간격으로 검사하고, **연속 두 번** 통과해야 확정합니다. 가장 빠른 확정 시점은 7초입니다.

최초 60초 안에 수렴하지 않은 조합은 **같은 영 초기상태·동역학·수렴 기준**으로 최대 300초까지 추가 계산합니다. 기존 60초 결과는 별도로 보존합니다. 시간만 늘려도 수렴하지 않거나 hard guard로 종료되는 조합은 유효한 수렴값으로 채우지 않습니다. 기본 조합 수 85,050개와 별도로 추가 계산 횟수를 검증 요약에 기록합니다.

각 검사에서는 마지막 2초 동안 동적 상태 `x(1:29)` 전체의 최대–최소 변동폭, 원시 RHS의 최대 절댓값, 인접한 두 구간의 평균 차이를 확인합니다. 방향각과 세계좌표 `x(30:32)`는 정상선회 중에도 변하므로 수렴 검사에서 제외합니다. 적분 후 상태 클리핑으로 정지한 것처럼 보이는 구간은 수렴으로 인정하지 않습니다.

허용값은 횡속도 `1e-4 m/s`, 요레이트·롤/피치각 `1e-5`의 각 상태 단위, 높이·수직속도·각속도·슬립 상태 `1e-5`의 각 상태 단위, 휠 속도 `1e-3 rad/s`, 횡가속도 필터 `1e-4 m/s²`입니다. RHS에는 같은 수치를 초당 변화율 기준으로 사용합니다. 구간 간 평균 차이는 상태 허용값의 절반 이하여야 합니다. 정확한 29개 상태별 값은 MAT에 저장합니다. 이 허용값은 **이 계산의 수치 판정 기준**입니다.

이는 영 초기조건에서 관측한 정상상태이며 전체 초기상태 격자의 유일한 수렴점이나 전역적 안정성을 보장하지 않습니다. 조향각 0°도 MF 오프셋 때문에 출력값이 정확히 0이 아닐 수 있습니다.

## 주요 MAT 필드

| 필드 | 의미 |
|---|---|
| `vc_ss`, `r_ss`, `phi_ss` | 수렴한 마지막 2초의 평균값, 단위 `m/s`, `rad/s`, `rad` |
| `converged` | 동적 상태의 수치 수렴 판정 |
| `tail_constraint_safe` | 마지막 2초 구간이 완전하고 기존 yaw·ay·LTR·수치·hard guard 제약을 만족 |
| `usable` | `converged && tail_constraint_safe` |
| `safe_vc_ss`, `safe_r_ss`, `safe_phi_ss` | `usable`인 조합만 값을 보존한 맵 |
| `all_observed_samples_constraint_safe` | 관측한 전체 과도응답에서 제약 위반이 없었는지 |
| `status` | 아래의 상태 코드 |
| `computed`, `complete`, `completed_count` | 실제 계산 여부 및 전체 완료 상태 |
| `reason_total`, `reason_tail`, `reason_first5s` | 전체·마지막 구간·최초 5초의 원인 비트 |
| `first5s_complete` | 최초 5초 전체를 계산했는지; 중간 종료 시 false |
| `end_time_s` | 수렴 또는 종료까지의 시간 |
| `max_scaled_ptp`, `max_scaled_rhs`, `max_scaled_mean_drift` | 상태 허용값으로 나눈 구간 최대 진단량 |
| `diagnostics.final_state` | 85,050행 × 32열의 마지막 전체 상태 |
| `provenance` | 원본·새 엔진 소스 해시, 실행 설정과 계산의 범위 |

`status`는 `0=미계산`, `1=수렴`, `2=시간 한도 내 미수렴`, `3=수치 실패`, `4=hard guard 종료`입니다. 미수렴·실패·미계산의 정상상태 출력은 **NaN**입니다. 수렴했지만 기존 안정 제약을 초과한 조합은 `converged=true`, `usable=false`로 구분합니다. 이러한 값은 `safe_*` 맵에서 NaN입니다.

원인 비트는 `1=yaw rate`, `2=횡가속도`, `4=LTR`, `8=수치 비유한값`, `16=hard state guard`입니다. 속도·조향각·μ 사이를 보간하거나 NaN인 조건을 주변 값으로 채우는 작업은 수행하지 않았습니다. 주요 맵의 속도·μ·조향각 축은 LUT 도구에서 사용하기 쉽도록 모두 **오름차순**입니다. 마모와 서스펜션 모드는 범주형 선택값이므로 이 두 축 사이를 보간하지 마세요.

## MATLAB 조회 예

```matlab
S = load('steady_state_lookup_210x5x81.mat');
L = S.STEADY_STATE_LUT;
assert(L.complete, '아직 전체 계산이 완료되지 않은 체크포인트입니다.');
is = find(L.speed_kmh == 80);
im = find(abs(L.mu_road - 0.9) < 1e-12);
id = find(L.front_wheel_steer_deg == 2);
iw = 1;  % 정상 타이어
ik = find(strcmp(L.suspension_modes, 'nnnn'));
idx = sub2ind(size(L.vc_ss), is, im, id, iw, ik);
fprintf('status=%d, converged=%d, usable=%d\n', ...
    L.status(idx), L.converged(idx), L.usable(idx));
value = [L.vc_ss(idx), L.r_ss(idx), L.phi_ss(idx)]
% 안전 제약을 만족한 수렴값만 사용할 때:
safe_value = [L.safe_vc_ss(idx), L.safe_r_ss(idx), L.safe_phi_ss(idx)]
```

`diagnostics.final_state`의 행은 엔진의 0부터 시작하는 `row_id` 순서이고, 서스펜션 모드 → 마모 → 조향각 → μ → 속도 순으로 변합니다. **엔진의 μ 순서는 내림차순**이며 주요 맵 배열의 μ 오름차순과 다릅니다. 주요 맵의 한 조합에 해당하는 전체 상태는 `row = L.row_id_zero_based_map(idx) + 1; x = L.diagnostics.final_state(row,:);`로 조회합니다. MATLAB 배열의 선형 인덱스를 그대로 진단 행에 대입하지 마세요.

## 같은 계산 재실행

Python 3, NumPy, SciPy 및 OpenMP를 지원하는 C++17 컴파일러가 필요합니다. 저장소 루트에서 다음 순서로 실행합니다. 계산 도중 중단하면 같은 명령으로 완료된 조건부터 이어서 실행합니다.

```bash
mkdir -p validation/steady_state_lookup
g++ -O3 -std=c++17 -fopenmp \
  steady_state_lookup/native_steady_lookup.cpp \
  -o validation/steady_state_lookup/native_steady_lookup
python steady_state_lookup/run_lookup.py --threads 4 --max-time 60
```

진행 상태는 `results/steady_state_lookup/progress.json`에 기록합니다. 각 210조건의 마모 5종·81모드가 모두 끝나면 해당 조건을 완료로 집계합니다. 실행 파일이나 수치 설정이 바뀌면 기존 실행 캐시의 재사용을 거부합니다.

60초 안에 미수렴한 조합을 추가 확인하려면 기본 계산 완료 후 실행합니다.

```bash
cp validation/steady_state_lookup/run/rows.jsonl \
  validation/steady_state_lookup/run/rows_60s.jsonl
python steady_state_lookup/extend_time_limits.py \
  --input validation/steady_state_lookup/run/rows_60s.jsonl \
  --output validation/steady_state_lookup/run/rows.jsonl \
  --max-time 300 --threads 4
```

## 재패키징

시뮬레이션 JSONL을 생성한 후 다음 명령으로 MAT를 만듭니다. 빌더는 중복·범위 밖 행, 입력과 인덱스 불일치, 잘못된 모드 강성, 수렴 기준 미달의 수렴 표시를 거부하고 저장 후 MAT를 다시 읽어 검증합니다.

```bash
python steady_state_lookup/build_lookup.py \
  --input validation/steady_state_lookup/run/rows.jsonl \
  --run-metadata validation/steady_state_lookup/run/final_run_metadata.json \
  --output results/steady_state_lookup/steady_state_lookup_210x5x81.mat
```

전체 85,050행이 없으면 기본적으로 저장을 거부합니다. 진행 중 확인용으로만 `--allow-partial`을 사용하면 미계산 조합을 `status=0`, 출력 NaN으로 남기고 `complete=false`인 MAT를 만들 수 있습니다. 빌더의 읽기·저장 검증은 동역학의 독립 재시뮬레이션 검증과 별개이며, 수치 판정 기준을 통과하도록 결과값을 변경하지 않습니다.

300초 추가 확인을 생략한 재실행에서는 `final_run_metadata.json` 대신 기본 계산의 `run_fingerprint.json`을 지정합니다.
