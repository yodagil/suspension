# 안정영역 MAT 결과

**전체 210개 조건 계산과 자동 업로드는 중단되어 있습니다.** 아래 수정 결과만 새 보정으로 계산했습니다. 나머지 기존 결과는 이전 마찰력 정규화 방식이므로 수정 결과와 구분해서 사용해 주세요.

수정 코드는 기준 마찰계수 0.9에서 원본 첫 번째 코드의 타이어 힘을 그대로 유지하도록 종·횡방향 배율을 `mu/0.9`로 적용합니다. MATLAB 기준 파일과 같은 대칭 격자와 5초 전체 판정 코드를 사용합니다.

| 수정 완료 조건 | MAT 다운로드 | 검증 |
|---|---|---|
| 80 km/h · μ=0.9 · 2° | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_80kph_0.9mu_2deg.mat) | [기준 파일과 마모 5종 전체 안정/불안정 판정·좌표·영역 일치](corrected_reference09/reference_comparison.json) |

각 결과는 마모 5종, 마모별 초기조건 39,401개(41×31×31), 5초, Ts=0.01초, RK4 하위 적분 20회를 유지합니다. 실행 엔진은 GNU Octave/C++이며 MAT는 MATLAB에서 읽을 수 있습니다. `CaseSummary`와 `CaseRanking`은 이식성을 위한 구조체이므로 원본 MATLAB table의 저장 형식과는 다릅니다.

안정점은 5초 동안 설정 제약을 만족한 초기조건이며 점근 안정성을 증명하는 것은 아닙니다. 수정 원인과 보정 설명은 [보정 기록](corrected_reference09/CALIBRATION_CHANGE.md)을 참고하세요.

## 이전 방식 결과 보관

이전 방식으로 저장한 38개 조건의 목록입니다. 위에서 수정 완료한 조건의 이전 파일은 `legacy_absolute_peak_v1`에 보존했습니다. 나머지는 아직 재계산하지 않았습니다.

| 속도 (km/h) | 마찰 설정 | 앞 휠 조향각 (deg) | 이전 MAT |
|---|---|---|---|
| 50 | 0.5 | 0.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.5mu_0.5deg.mat) |
| 50 | 0.5 | 0 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.5mu_0deg.mat) |
| 50 | 0.5 | 1.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.5mu_1.5deg.mat) |
| 50 | 0.5 | 1 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.5mu_1deg.mat) |
| 50 | 0.5 | 2.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.5mu_2.5deg.mat) |
| 50 | 0.5 | 2 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.5mu_2deg.mat) |
| 50 | 0.5 | 3 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.5mu_3deg.mat) |
| 50 | 0.6 | 0.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.6mu_0.5deg.mat) |
| 50 | 0.6 | 0 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.6mu_0deg.mat) |
| 50 | 0.6 | 1.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.6mu_1.5deg.mat) |
| 50 | 0.6 | 1 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.6mu_1deg.mat) |
| 50 | 0.6 | 2.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.6mu_2.5deg.mat) |
| 50 | 0.6 | 2 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.6mu_2deg.mat) |
| 50 | 0.6 | 3 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.6mu_3deg.mat) |
| 50 | 0.7 | 0.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.7mu_0.5deg.mat) |
| 50 | 0.7 | 0 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.7mu_0deg.mat) |
| 50 | 0.7 | 1.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.7mu_1.5deg.mat) |
| 50 | 0.7 | 1 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.7mu_1deg.mat) |
| 50 | 0.7 | 2.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.7mu_2.5deg.mat) |
| 50 | 0.7 | 2 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.7mu_2deg.mat) |
| 50 | 0.7 | 3 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.7mu_3deg.mat) |
| 50 | 0.8 | 0.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.8mu_0.5deg.mat) |
| 50 | 0.8 | 0 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.8mu_0deg.mat) |
| 50 | 0.8 | 1.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.8mu_1.5deg.mat) |
| 50 | 0.8 | 1 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.8mu_1deg.mat) |
| 50 | 0.8 | 2.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.8mu_2.5deg.mat) |
| 50 | 0.8 | 2 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.8mu_2deg.mat) |
| 50 | 0.8 | 3 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.8mu_3deg.mat) |
| 50 | 0.9 | 0.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_0.5deg.mat) |
| 50 | 0.9 | 0 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_0deg.mat) |
| 50 | 0.9 | 1.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_1.5deg.mat) |
| 50 | 0.9 | 1 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_1deg.mat) |
| 50 | 0.9 | 2.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_2.5deg.mat) |
| 50 | 0.9 | 2 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_2deg.mat) |
| 50 | 0.9 | 3 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_3deg.mat) |
| 60 | 0.9 | 0.5 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_60kph_0.9mu_0.5deg.mat) |
| 60 | 0.9 | 0 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_60kph_0.9mu_0deg.mat) |
| 80 | 0.9 | 2 | [이전 결과](https://raw.githubusercontent.com/yodagil/suspension/main/results/legacy_absolute_peak_v1/stable_point_80kph_0.9mu_2deg.mat) |
