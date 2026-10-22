# 안정영역 MAT 결과

**전체 210개 조건 계산과 자동 업로드는 중단되어 있습니다.** 이전 방식의 MAT 파일은 현재 GitHub 목록에서 제거했고, 검증을 마친 수정 결과만 다시 업로드합니다.

수정 코드는 기준 마찰계수 0.9에서 원본 첫 번째 코드의 타이어 힘을 그대로 유지하도록 종·횡방향 배율을 `mu/0.9`로 적용합니다. MATLAB 기준 파일과 같은 대칭 격자와 5초 전체 판정 코드를 사용합니다.

| 수정 완료 조건 | MAT 다운로드 | 검증 |
|---|---|---|
| 80 km/h · μ=0.9 · 2° | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_80kph_0.9mu_2deg.mat) | [기준 파일과 마모 5종 전체 안정/불안정 판정·좌표·영역 일치](corrected_reference09/reference_comparison.json) |
| 50 km/h · μ=0.9 · 0° | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_0deg.mat) | [동일 수정 코드 재계산 및 저장 데이터 검증 완료; 별도 50 km/h 기준 파일 없음](corrected_reference09_50kph/validation_comparison.json) |

각 결과는 마모 5종, 마모별 초기조건 39,401개(41×31×31), 5초, Ts=0.01초, RK4 하위 적분 20회를 유지합니다. 실행 엔진은 GNU Octave/C++이며 MAT는 MATLAB에서 읽을 수 있습니다. `CaseSummary`와 `CaseRanking`은 이식성을 위한 구조체이므로 원본 MATLAB table의 저장 형식과는 다릅니다.

안정점은 5초 동안 설정 제약을 만족한 초기조건이며 점근 안정성을 증명하는 것은 아닙니다. 수정 원인과 보정 설명은 [보정 기록](corrected_reference09/CALIBRATION_CHANGE.md)을 참고하세요.
