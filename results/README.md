# 수정된 안정영역 MAT 결과

<!-- KINEMATIC-BEGIN -->
현재 새 초기화 방식의 정상 타이어 우선 계산은 **1/3개 완료·검증**했습니다. [새 결과와 다운로드](kinematic_initialization_healthy/README.md)를 확인하세요. 아래 목록은 이전 직접 초기화 방식의 결과입니다.
<!-- KINEMATIC-END -->

**이전 직접 초기화 방식(v3): 11/210개 저장. 사용자 요청으로 일괄 계산을 중단했습니다.**

실험 조건: 속도 50·60·70·80·90·100 km/h, 노면 마찰계수 0.9·0.8·0.7·0.6·0.5, 앞 휠 조향각 0·0.5·1·1.5·2·2.5·3°. 바퀴 조향각은 [angle, angle, 0, 0]입니다.

원본의 마모 5종, 마모별 초기조건 39,401개(41×31×31), 5초, Ts=0.01초, RK4 하위 적분 20회를 유지합니다. 실행 엔진은 GNU Octave/C++이며 MATLAB에서 읽을 수 있는 MAT v7 파일을 저장합니다.

타이어 힘 배율은 mu/0.9이며 mu=0.9에서 첫 번째 원본 코드의 보정을 유지합니다. [80 km/h·mu=0.9·2° 기준 파일 재현 검증](corrected_reference09/reference_comparison.json)은 마모 5종 전체 판정·좌표·영역이 일치했습니다. [50 km/h·mu=0.9·0° 재계산 검증](corrected_reference09_50kph/validation_comparison.json)도 완료했으며 별도 50 km/h 기준 파일은 제공되지 않았습니다.

안정점은 5초 동안 설정 제약을 만족한 초기조건입니다. 유효한 내부 다면체를 얻지 못한 경우 원시 판정과 좌표는 보존하고 영역 부피는 NaN으로 기록합니다. 구조체 요약은 원본 MATLAB table의 저장 형식과 다릅니다. [전체 공개 파일 검증 요약](reference09_publication.json)

| 속도 (km/h) | mu | 앞 휠 조향각 (deg) | MAT 다운로드 | 유효 내부 영역 (마모 5종) |
|---|---|---|---|---|
| 50 | 0.9 | 0 | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_0deg.mat) | 5/5 |
| 50 | 0.9 | 0.5 | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_0.5deg.mat) | 5/5 |
| 50 | 0.9 | 1 | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_1deg.mat) | 5/5 |
| 50 | 0.9 | 1.5 | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_1.5deg.mat) | 5/5 |
| 50 | 0.9 | 2 | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_2deg.mat) | 5/5 |
| 50 | 0.9 | 2.5 | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_2.5deg.mat) | 5/5 |
| 50 | 0.9 | 3 | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.9mu_3deg.mat) | 3/5 |
| 50 | 0.8 | 0 | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.8mu_0deg.mat) | 5/5 |
| 50 | 0.8 | 0.5 | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.8mu_0.5deg.mat) | 5/5 |
| 50 | 0.8 | 1 | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_50kph_0.8mu_1deg.mat) | 5/5 |
| 80 | 0.9 | 2 | [다운로드](https://raw.githubusercontent.com/yodagil/suspension/main/results/stable_point_80kph_0.9mu_2deg.mat) | 5/5 |
