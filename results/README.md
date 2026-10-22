# 수정된 안정영역 MAT 결과

기존 방식으로 계산한 MAT 파일은 GitHub 최신 파일 목록에서 제거했습니다. 아래에는 수정 코드로 다시 계산하고 검증한 결과만 올립니다.

| 조건 | MAT 다운로드 | 검증 |
|---|---|---|
| 80 km/h · μ=0.9 · 앞 휠 조향각 2° | [stable_point_80kph_0.9mu_2deg.mat](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_80kph_0.9mu_2deg.mat) | [원본 기준 파일과 마모 5종 전체 일치](corrected_reference09/reference_comparison.json) |

50 km/h · μ=0.9 · 앞 휠 조향각 0°는 같은 수정 코드로 재계산 중입니다. 완료 및 검증 후 업로드합니다. 전체 210개 조건 일괄 계산은 중단 상태입니다.

각 결과는 마모 5종, 마모별 초기조건 39,401개(41×31×31), 5초, Ts=0.01초, RK4 하위 적분 20회를 유지합니다. 실행 엔진은 GNU Octave/C++이며 MATLAB에서 읽을 수 있는 MAT v7 형식으로 저장합니다.

80 km/h 결과는 197,005개 초기조건의 안정/불안정 마스크와 누적 원인 코드가 기준 파일과 전부 일치하며, 좌표 및 내부·외부 영역도 일치합니다. 원본 MATLAB table은 이식성을 위해 구조체로 저장하므로 파일의 바이트 및 저장 형식은 다릅니다.

종·횡 타이어 힘 배율을 `mu/0.9`로 적용하여 μ=0.9에서 원본 첫 번째 코드의 타이어 보정을 유지했습니다. [보정 설명](corrected_reference09/CALIBRATION_CHANGE.md)을 참고하세요. 안정점은 5초 동안 설정 제약을 만족한 초기조건을 뜻합니다.
