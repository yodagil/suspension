# CDC 연구 최종 코드: 210조건 × 마모 5종

사용자가 제공한 `CDC_research_final_code_results.zip`의 MATLAB 15개와 Native 소스 2개를 수정 없이 사용합니다. 기존 V2 결과는 제거하고, 새 계산은 `../results/cdc_research_final`에 저장합니다.

## MATLAB에서 실행

이 폴더를 MATLAB Current Folder로 설정합니다.

```matlab
run_cdc_research('selftest');
% MATLAB M-code 엔진: 별도 컴파일 불필요
R = run_cdc_research('full');
```

동일한 모델의 가속 실행은 C++17/OpenMP 지원 컴파일러가 있을 때 가능합니다.

```matlab
cdc_build_native;
R = run_cdc_research('full', struct('engine','native'));
```

기본 저장 위치는 이 폴더의 `results_new_full`입니다. 지정하려면 다음처럼 실행합니다.

```matlab
R = run_cdc_research('full', struct('engine','native', ...
    'output_dir', fullfile(pwd,'results_210')));
```

이 서버에서는 GNU Octave 9.4 + Native C++ 엔진, 4스레드를 사용합니다. 원본 selftest와 독립적인 10초 M-code/Native 비교를 통과했습니다. Native 실행의 판정과 진단을 독립적으로 비교했고, 정상 복귀하더라도 H를 이탈한 점은 `final`에서 제외됨을 확인했습니다.

## 계산 조건

- 차속: 50, 60, 70, 80, 90, 100 km/h, 각 시뮬레이션에서 고정.
- 노면 마찰계수: 0.9, 0.8, 0.7, 0.6, 0.5.
- 앞바퀴 조향각: 0, 0.5, 1, 1.5, 2, 2.5, 3°. 뒷바퀴 0°.
- 타이어 마모 `[FL FR RL RR]`: `[0 0 0 0]`, `[25 25 0 0]`, `[50 50 25 25]`, `[0 0 25 25]`, `[25 25 50 50]`.
- 네 코너 스프링 30,790 N/m, CDC 950 mA, MRf=0.8613, MRr=0.7970. 추가 피드백 없음.
- 초기 횡속도 −3:0.1:3 m/s, 초기 요레이트 −1.5:0.025:1.5 rad/s, 61×121개 격자.
- 각 마모의 계산된 정상선회점(trim)으로 롤각과 다른 숨은 상태를 초기화하고, 횡속도와 요레이트만 바꿉니다. 32상태 모델의 롤·현가 동역학은 계속 적분합니다.
- 시간간격 0.0005초, 동일 궤적에서 5초·10초 판정, 마지막 1초 구간의 상태·미분 오차로 복귀 판정.

전체 격자 후보는 7,750,050개이지만 **정적 handling 기준 영역 H 안의 초기점만 실제 적분**합니다. 1,050개 조건·마모 조합 사전 계산에서 H 초기점은 총 841,284개이며, 영역 추출을 위한 셀 중심점도 별도로 적분합니다. H 밖의 점은 미계산이고 불안정으로 분류하면 안 됩니다.

## 새 판정과 MAT 구조

MAT 변수명은 `RESEARCH2D`입니다. `RESEARCH2D.CASE{1}`부터 `{5}`까지가 타이어 마모 5종입니다.

```matlab
load('research_50kph_mu0.9_delta+0_I950_950_950_950_trim.mat');
C = RESEARCH2D.CASE{1};
H = C.masks.reference;       % 61×121, 초기 handling 기준
F5 = C.masks.final(:,:,1);   % 5초 H 유지 복귀 판정
F10 = C.masks.final(:,:,2);  % 10초 H 유지 복귀 판정
cells = C.region.cell_mask; % 60×120, 네 꼭짓점+중심점 통과 셀
area = C.region.cell_area;
plot_cdc_research({'research_50kph_mu0.9_delta+0_I950_950_950_950_trim.mat'},1,'final');
```

`plot_cdc_research`의 입력 형식은 원본 함수의 도움말을 확인하세요. 저장 파일을 그리려면 `plot_cdc_research({'파일경로.mat'},1,'final')`을 사용합니다.

주 결과 **`masks.final`**은 H 안의 초기점 중 경로 제약을 만족하고, 궤적이 H를 한 번도 벗어나지 않으며, 정상선회점으로 복귀한 점입니다. 목표 trim이 H 밖이면 주 결과를 비웁니다.

- `recoverable_on_reference`: H 안에서 시작하여 복귀. 도중 H 이탈을 허용하는 비교 결과.
- `guarded_final`: final에 절대 요레이트 정책을 추가한 결과.
- `prepeak_final`: final에 동적 타이어 복원기울기 양수 조건을 추가한 결과.
- `raw`: H 안에서 실제 계산한 초기좌표, 1-based 격자 인덱스, 상태·복귀·H 이탈·안전 진단.
- `region`: 10초 final의 네 꼭짓점이 통과한 셀에 대해 실제 중심점도 계산해 얻은 영역. 중심점 원시 진단도 저장.
- `source`: 실제 계산에 사용한 MATLAB/Native 소스 본문.

요레이트 정책·횡가속도 정책·H 이탈은 기록하며, 하드 모델 도메인 위반과 달리 그 자체로 적분을 중단하지 않습니다. 복귀와 경로 제약을 구분해 저장합니다. 결과는 유한시간 표본 판정으로, 연속영역·불변집합·점근 안정영역의 증명은 아닙니다. 볼록껍질 채우기·이동·억지 포함 제약은 사용하지 않습니다.

## 서버 자동 실행 및 업로드

- `run_full_210.py`: 소스/Native 실행 파일을 동결하고 SHA-256을 확인하며 실행. 마모별 체크포인트로 이어서 계산.
- `validate_results.py`: 원본 소스 해시, 조건, H 초기점, 판정과 원시 진단, 실제 셀 중심 검사 및 면적을 교차 검증.
- `plot_results.py`: 저장된 결과만 사용해 조건별 마모 5종 그림과 속도·마찰계수별 조향각 7종 비교 그림을 생성.
- `publish_results.py`: 검증된 최종 MAT와 출처가 일치하는 그림을 개별 완료 때 GitHub에 게시. 체크포인트·실행 바이너리는 게시하지 않음.

예상 총 계산시간은 실제 네 조건·마모 벤치마크와 전체 H 크기를 반영해 약 **10~15시간**입니다. 환경·조건별 연산량에 따라 달라집니다. 공개 `publication_progress.json`은 업로드 당시 상태이며, `publication_summary.csv`는 마모별 수치 요약입니다.
