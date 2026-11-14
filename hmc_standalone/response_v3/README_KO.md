# 독립 Full-car 응답 시뮬레이션

[단일 M 파일](HMC_FullCar_Compare.m) · [코드 ZIP](HMC_FullCar_Compare_MATLAB.zip)

`HMC_FullCar_Compare.m`은 선택한 차량 동역학·MF 타이어 계산식, CDC 맵 처리,
자체 정상상태 계산, RK4 적분기와 그래프 저장 기능을 한 MATLAB 파일에 포함합니다.
모델 버전: `HMC_TRANSIENT_RESPONSE_V3`.

원본 차량·타이어·댐퍼 계수는 파일에 포함하지 않았습니다. 이미 보유한 원본 입력을
아래처럼 로컬에서 읽으세요. 계산에는 CarSim 실행기, MF-Swift DLL 또는
Simulink 실행이 필요하지 않습니다. SLX는 댐퍼 맵을 읽는 데이터 파일로만 사용합니다.
ZIP/SLX 입력을 읽을 때 MATLAB의 일반 JVM 실행 환경을 사용합니다.

## 1. 로컬 입력 준비

이 `.m` 파일을 MATLAB 폴더에 놓고 다음을 실행하세요.
경로는 본인 PC의 실제 파일 위치로 바꿉니다.

```matlab
cfg = struct;
cfg.echo_par = fullfile(pwd,'trim_echo.par');
cfg.tire_zip = fullfile(pwd,'HMC_wear_model.zip'); % 또는 압축 해제 폴더
cfg.slx = fullfile(pwd,'HMC_PHM.slx');
cfg.output_mat = fullfile(pwd,'hmc_data.mat');
HMC_FullCar_Compare('import',cfg);
```

`trim_end.par`나 CarSim에서 얻은 정상상태 스냅샷은 입력하지 않습니다.
가져온 `hmc_data.mat`은 본인 PC에서 사용하며 원본 계수도 포함합니다.

## 2. 짧은 검사와 실행

```matlab
opts = struct('data_file',fullfile(pwd,'hmc_data.mat'));
testOpts = opts;
testOpts.speed_kmh = 50;
testOpts.mu = 0.5;
testOpts.steer_deg = 0;
HMC_FullCar_Compare('selftest',testOpts); % 자체 정상상태 계산 및 5 ms 적분 검사

R = HMC_FullCar_Compare('run',opts);   % 기본: 90 km/h, mu=0.6, 조향 명령 2.5 deg
R = HMC_FullCar_Compare('three',opts); % 50/.5/0, 70/.6/2.5, 90/.6/2.5
```

세 조건 모두 정상 타이어, 네 바퀴 정상 스프링 모드, CDC 전류 950 mA입니다.
초기 횡속도는 +0.1 m/s, 초기 body yaw rate는 +0.35 rad/s입니다.
각 조건의 다른 초기 상태는 이 독립 모델이 직접 계산한 정상상태를 유지하며,
내부 타이어 이완 상태는 검증한 재시작 정책에 따라 0으로 설정합니다.
정상상태 계산은 비선형 평형식의 수치적 근을 찾는 방식으로, 실제 12초간 trim을
시간 적분하거나 CarSim의 정상상태를 주입하는 방식이 아닙니다.
첫 정상상태 계산과 조향각 연속 해석에는 수 분이 걸릴 수 있습니다.
이 계산을 마친 뒤 적분 시간 진행 표시가 시작됩니다. 실제 소요 시간은 MATLAB과 PC에 따라 다릅니다.

```matlab
opts.speed_kmh = 70;
opts.mu = 0.6;
opts.steer_deg = 2.5;
opts.vc0_mps = 0.1;
opts.r0_radps = 0.35;
opts.duration = 10;
opts.dt = 0.001;
opts.output_dir = fullfile(pwd,'results_response');
R = HMC_FullCar_Compare('run',opts);
```

적분은 순수 M-code의 고정 간격 RK4입니다. CDC 명령은 1 ms마다 갱신되고
2 ms 지연 후 적용됩니다. 한 RK4 스텝 안에서는 적용 힘을 유지합니다.
모델의 PI 종방향 속도 제어를 사용하므로 종방향 속도가 완전히 상수인 모델은 아닙니다.

## 3. CarSim 시간 응답과 비교

원본 `CarSimOut_trim` timeseries가 담긴 로컬 MAT를 지정하면 8개 응답을 겹쳐 그립니다.

```matlab
opts.reference_file = fullfile(pwd,'CarSimOut_trim_90.mat');
R = HMC_FullCar_Compare('run',opts);

opts.reference_files = {fullfile(pwd,'CarSimOut_trim_50.mat'), ...
                       fullfile(pwd,'CarSimOut_trim_70.mat'), ...
                       fullfile(pwd,'CarSimOut_trim_90.mat')};
R = HMC_FullCar_Compare('three',opts);
```

비교 신호는 횡속도, body yaw rate, roll angle, pitch angle, body roll rate p,
body pitch rate q, 종방향 속도, 횡가속도입니다. p와 q는 상태 11·12이며
Euler 각도를 미분한 값과 다릅니다. 정상 선회에서 Euler pitch angle이 일정하더라도
body pitch rate q는 0이 아닐 수 있습니다.
초기 과도 구간을 포함한 모든 원본 샘플을 비교하며, 시간 이동이나 파형 보정은 하지 않습니다.

## 4. 출력과 검증 범위

각 조건의 계산이 끝나면 `results_response`에 응답 MAT 및 8패널 PNG가 저장됩니다.
MAT에는 선택한 응답, 계산된 힘, 조건, 평형 수렴 상태, 비교 지표가 들어갑니다.
차량·타이어의 원본 계수 구조체 Q, 정상상태 전체 구조체 T, 입력 구조체 c는 저장하지 않습니다.

이 파일은 독립 물리 근사 모델입니다. CarSim/MF-Swift 내부 방정식 전체를
복제한 모델이 아니므로 모든 운전 조건에서 같은 결과를 보장하지 않습니다.
순수 M-code는 네이티브 엔진보다 느립니다. 클라우드의 10초 응답은 같은 선택 모델의
네이티브 엔진으로 계산했으며, 이 단일 M-code 파일은 RHS 및 짧은 적분 동등성 검사로
확인합니다. 이 전달 파일로 10초 전체를 적분 완료했다고 주장하지 않습니다.
정확한 소스 해시와 선택 옵션은 `build_manifest.json`에 있습니다.
