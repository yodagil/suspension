# stability3d_wear: 4조건 MAT 결과

완료 및 업로드: 2/4개. 각 파일에 마모 5종을 포함합니다.

첨부된 `stability3d_wear.m`의 revised 모델로 계산했습니다. 기존 공개 MAT와 다른 모델이며 결과를 혼합하지 않습니다.

- 사용자 지정 기본 격자: vc −10~10 (41개), r −0.4~0.4 (31개), phi −0.4~0.4 (31개).
- 마모당 39,401개, 파일당 197,005개, 전체 788,020개 격자 초기조건. 별도의 영역 검증점은 이 수에 포함하지 않습니다.
- 5초, Ts=0.01초, RK4 하위 적분 20회(dt=0.0005초), 매 하위 적분에서 제약 검사.
- source 타이어 힘, revised 초기화, roll gravity sign +1, mu_limit=0.9. early_exit=true이므로 불안정 조건은 첫 중단까지 원인 비트를 누적합니다.
- off-grid 영역 검사와 독립 holdout은 원본 기본 설정대로 유지하고 dt를 절반으로 사용했습니다.
- 그래프만 비활성화했습니다. 격자 수·시뮬레이션 시간·영역 검증 표본 수는 축소하지 않았습니다.
- GNU Octave 9.4 및 C++ 가속 경로 사용. 원본 궤적 120개에 대한 분류/진단/최종 상태 대조 통과. 전체 5초 궤적 대조 기록은 실행 검증에 포함됩니다.
- Octave 문법 호환 수정: all(A,'all') → all(all(A)). 자체 테스트의 yaw limit 예상값 2곳은 코드의 실제 식으로 교정했습니다. 물리 계산식은 바꾸지 않았습니다.
- MAT v7 형식. 체크포인트/중간 MAT는 게시하지 않습니다. 수치 검증 결과와 SHA-256은 validation.json에 기록합니다.

MATLAB에서 `D=load(filename); S=D.STABILITY3D;`로 읽습니다. `S.CASE{1}`~`S.CASE{5}`가 마모별 결과이며 `S.scenario`가 해당 운행 조건입니다. `cfg.scenarios`는 원본의 전체 4조건 목록이고 `scenario_index`가 이 파일의 해당 행입니다.

영역의 `empirical_valid=false`이면 검증된 내부 영역을 얻지 못한 것입니다. 이 경우 부피 NaN과 실패 이유를 보존하며 안정/불안정 격자 분류 데이터는 사용할 수 있습니다. 검증 통과 영역도 연속 안전집합이나 불변집합의 증명은 아닙니다.

| 조건 | MAT | 안정점 수 (마모 1~5) | 검증된 내부 영역 |
|---|---|---|---|
| 80 km/h · 0° | [stability3d_wear_80kph_0deg.mat](stability3d_wear_80kph_0deg.mat) | 2568, 2462, 2853, 2461, 2458 | 5/5 |
| 80 km/h · 2° | [stability3d_wear_80kph_2deg.mat](stability3d_wear_80kph_2deg.mat) | 3011, 3031, 3435, 2865, 3334 | 5/5 |
