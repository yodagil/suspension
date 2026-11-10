# 초기 롤각 0의 2차원 CDC 안전·복귀 영역

**검증 후 업로드: 186/210조건 · 조건별 마모 5종.** 계산 상태: `running`.

[현재 진행 상황](publication_progress.json) · [전체 수치 요약](publication_summary.csv) · [실행·판정 설명](../../cdc_shift_2d/README_KO.md)

초기조건은 횡속도 41개 × 요레이트 31개입니다. **초기 롤각만 0으로 고정**하고 32상태 모델의 롤 운동은 계속 적분합니다. 3차원 영역을 투영하거나 데이터를 평행이동한 결과가 아닙니다.

첨부 CDC 코드의 trim 초기화와 5초·10초 판정을 유지합니다. `safe`, `guarded_safe`, `recoverable`, `safe_unresolved`를 따로 저장하며 대표 선택 영역은 10초 복귀와 요레이트 정책의 교집합입니다. 유한시간 표본 판정이며 증명된 점근 안정영역이나 불변집합을 뜻하지 않습니다.

MAT 변수명은 `STABILITY2D`입니다. `stable_points`는 초기 `[vc0, r0]` 좌표이고, `region.sampled_area`는 네 꼭짓점과 요청된 중심점 검사를 통과한 2차원 셀 면적입니다. 격자 끝에 닿은 결과는 잘렸을 수 있으므로 아래 표시와 원시 마스크를 함께 확인하세요.

| 속도 km/h | μ | 앞 휠 조향각 ° | MAT | 2D 그림 | 선택점 수, 마모 1~5 | 격자 경계 접촉 |
|---|---|---|---|---|---|---|
| 50 | 0.9 | 0 | [MAT](cdc2d_50kph_mu0.9_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.9_delta+0_trim_phi00.png) | 122, 127, 312, 116, 185 | 1, 2, 3, 4, 5 |
| 50 | 0.9 | 0.5 | [MAT](cdc2d_50kph_mu0.9_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.9_delta+0.5_trim_phi00.png) | 127, 138, 319, 118, 197 | 1, 2, 3, 4, 5 |
| 50 | 0.9 | 1 | [MAT](cdc2d_50kph_mu0.9_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.9_delta+1_trim_phi00.png) | 145, 161, 340, 124, 209 | 1, 2, 3, 4, 5 |
| 50 | 0.9 | 1.5 | [MAT](cdc2d_50kph_mu0.9_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.9_delta+1.5_trim_phi00.png) | 174, 193, 371, 146, 241 | 1, 2, 3, 4, 5 |
| 50 | 0.9 | 2 | [MAT](cdc2d_50kph_mu0.9_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.9_delta+2_trim_phi00.png) | 210, 231, 399, 176, 285 | 1, 2, 3, 4, 5 |
| 50 | 0.9 | 2.5 | [MAT](cdc2d_50kph_mu0.9_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.9_delta+2.5_trim_phi00.png) | 245, 271, 436, 211, 325 | 1, 2, 3, 4, 5 |
| 50 | 0.9 | 3 | [MAT](cdc2d_50kph_mu0.9_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.9_delta+3_trim_phi00.png) | 279, 298, 483, 259, 383 | 1, 2, 3, 4, 5 |
| 50 | 0.8 | 0 | [MAT](cdc2d_50kph_mu0.8_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.8_delta+0_trim_phi00.png) | 112, 113, 202, 110, 128 | 1, 2, 3, 4, 5 |
| 50 | 0.8 | 0.5 | [MAT](cdc2d_50kph_mu0.8_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.8_delta+0.5_trim_phi00.png) | 115, 117, 197, 111, 131 | 1, 2, 3, 4, 5 |
| 50 | 0.8 | 1 | [MAT](cdc2d_50kph_mu0.8_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.8_delta+1_trim_phi00.png) | 120, 128, 205, 113, 146 | 1, 2, 3, 4, 5 |
| 50 | 0.8 | 1.5 | [MAT](cdc2d_50kph_mu0.8_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.8_delta+1.5_trim_phi00.png) | 133, 150, 218, 122, 164 | 1, 2, 3, 4, 5 |
| 50 | 0.8 | 2 | [MAT](cdc2d_50kph_mu0.8_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.8_delta+2_trim_phi00.png) | 161, 170, 267, 126, 198 | 1, 2, 3, 4, 5 |
| 50 | 0.8 | 2.5 | [MAT](cdc2d_50kph_mu0.8_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.8_delta+2.5_trim_phi00.png) | 192, 201, 324, 140, 240 | 1, 2, 3, 4, 5 |
| 50 | 0.8 | 3 | [MAT](cdc2d_50kph_mu0.8_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.8_delta+3_trim_phi00.png) | 219, 238, 373, 177, 293 | 1, 2, 3, 4, 5 |
| 50 | 0.7 | 0 | [MAT](cdc2d_50kph_mu0.7_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.7_delta+0_trim_phi00.png) | 107, 106, 140, 106, 104 | 1, 2, 3, 4, 5 |
| 50 | 0.7 | 0.5 | [MAT](cdc2d_50kph_mu0.7_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.7_delta+0.5_trim_phi00.png) | 108, 105, 116, 107, 106 | 1, 2, 3, 4, 5 |
| 50 | 0.7 | 1 | [MAT](cdc2d_50kph_mu0.7_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.7_delta+1_trim_phi00.png) | 108, 107, 111, 104, 114 | 1, 2, 3, 4, 5 |
| 50 | 0.7 | 1.5 | [MAT](cdc2d_50kph_mu0.7_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.7_delta+1.5_trim_phi00.png) | 111, 110, 123, 103, 128 | 1, 2, 3, 4, 5 |
| 50 | 0.7 | 2 | [MAT](cdc2d_50kph_mu0.7_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.7_delta+2_trim_phi00.png) | 117, 123, 170, 109, 146 | 1, 2, 3, 4, 5 |
| 50 | 0.7 | 2.5 | [MAT](cdc2d_50kph_mu0.7_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.7_delta+2.5_trim_phi00.png) | 140, 142, 208, 113, 164 | 1, 2, 3, 4, 5 |
| 50 | 0.7 | 3 | [MAT](cdc2d_50kph_mu0.7_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.7_delta+3_trim_phi00.png) | 179, 183, 257, 128, 197 | 1, 2, 3, 4, 5 |
| 50 | 0.6 | 0 | [MAT](cdc2d_50kph_mu0.6_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.6_delta+0_trim_phi00.png) | 83, 80, 75, 79, 70 | 없음 |
| 50 | 0.6 | 0.5 | [MAT](cdc2d_50kph_mu0.6_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.6_delta+0.5_trim_phi00.png) | 83, 78, 74, 79, 73 | 없음 |
| 50 | 0.6 | 1 | [MAT](cdc2d_50kph_mu0.6_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.6_delta+1_trim_phi00.png) | 79, 77, 75, 78, 76 | 없음 |
| 50 | 0.6 | 1.5 | [MAT](cdc2d_50kph_mu0.6_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.6_delta+1.5_trim_phi00.png) | 80, 79, 76, 79, 87 | 없음 |
| 50 | 0.6 | 2 | [MAT](cdc2d_50kph_mu0.6_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.6_delta+2_trim_phi00.png) | 83, 82, 88, 80, 104 | 5 |
| 50 | 0.6 | 2.5 | [MAT](cdc2d_50kph_mu0.6_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.6_delta+2.5_trim_phi00.png) | 98, 103, 116, 88, 127 | 2, 3, 5 |
| 50 | 0.6 | 3 | [MAT](cdc2d_50kph_mu0.6_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.6_delta+3_trim_phi00.png) | 127, 133, 152, 94, 146 | 1, 2, 3, 5 |
| 50 | 0.5 | 0 | [MAT](cdc2d_50kph_mu0.5_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.5_delta+0_trim_phi00.png) | 55, 52, 49, 52, 48 | 없음 |
| 50 | 0.5 | 0.5 | [MAT](cdc2d_50kph_mu0.5_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.5_delta+0.5_trim_phi00.png) | 55, 53, 49, 53, 49 | 없음 |
| 50 | 0.5 | 1 | [MAT](cdc2d_50kph_mu0.5_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.5_delta+1_trim_phi00.png) | 58, 57, 51, 59, 56 | 없음 |
| 50 | 0.5 | 1.5 | [MAT](cdc2d_50kph_mu0.5_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.5_delta+1.5_trim_phi00.png) | 60, 59, 59, 60, 68 | 없음 |
| 50 | 0.5 | 2 | [MAT](cdc2d_50kph_mu0.5_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.5_delta+2_trim_phi00.png) | 64, 62, 64, 62, 75 | 5 |
| 50 | 0.5 | 2.5 | [MAT](cdc2d_50kph_mu0.5_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.5_delta+2.5_trim_phi00.png) | 73, 69, 83, 66, 97 | 5 |
| 50 | 0.5 | 3 | [MAT](cdc2d_50kph_mu0.5_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_50kph_mu0.5_delta+3_trim_phi00.png) | 105, 65, 91, 65, 110 | 1, 3, 5 |
| 60 | 0.9 | 0 | [MAT](cdc2d_60kph_mu0.9_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.9_delta+0_trim_phi00.png) | 137, 132, 178, 130, 127 | 1, 2, 3, 4, 5 |
| 60 | 0.9 | 0.5 | [MAT](cdc2d_60kph_mu0.9_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.9_delta+0.5_trim_phi00.png) | 142, 135, 173, 131, 137 | 1, 2, 3, 4, 5 |
| 60 | 0.9 | 1 | [MAT](cdc2d_60kph_mu0.9_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.9_delta+1_trim_phi00.png) | 145, 146, 189, 140, 158 | 1, 2, 3, 4, 5 |
| 60 | 0.9 | 1.5 | [MAT](cdc2d_60kph_mu0.9_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.9_delta+1.5_trim_phi00.png) | 149, 154, 234, 142, 190 | 1, 2, 3, 4, 5 |
| 60 | 0.9 | 2 | [MAT](cdc2d_60kph_mu0.9_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.9_delta+2_trim_phi00.png) | 160, 169, 276, 150, 217 | 1, 2, 3, 4, 5 |
| 60 | 0.9 | 2.5 | [MAT](cdc2d_60kph_mu0.9_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.9_delta+2.5_trim_phi00.png) | 179, 208, 343, 162, 240 | 1, 2, 3, 4, 5 |
| 60 | 0.9 | 3 | [MAT](cdc2d_60kph_mu0.9_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.9_delta+3_trim_phi00.png) | 216, 246, 401, 175, 282 | 1, 2, 3, 4, 5 |
| 60 | 0.8 | 0 | [MAT](cdc2d_60kph_mu0.8_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.8_delta+0_trim_phi00.png) | 121, 120, 122, 120, 114 | 1, 2, 3, 4, 5 |
| 60 | 0.8 | 0.5 | [MAT](cdc2d_60kph_mu0.8_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.8_delta+0.5_trim_phi00.png) | 122, 119, 121, 119, 114 | 1, 2, 3, 4, 5 |
| 60 | 0.8 | 1 | [MAT](cdc2d_60kph_mu0.8_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.8_delta+1_trim_phi00.png) | 128, 122, 126, 125, 124 | 1, 2, 3, 4, 5 |
| 60 | 0.8 | 1.5 | [MAT](cdc2d_60kph_mu0.8_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.8_delta+1.5_trim_phi00.png) | 139, 138, 138, 137, 139 | 1, 2, 3, 4, 5 |
| 60 | 0.8 | 2 | [MAT](cdc2d_60kph_mu0.8_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.8_delta+2_trim_phi00.png) | 143, 148, 168, 142, 161 | 1, 2, 3, 4, 5 |
| 60 | 0.8 | 2.5 | [MAT](cdc2d_60kph_mu0.8_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.8_delta+2.5_trim_phi00.png) | 155, 158, 214, 149, 183 | 1, 2, 3, 4, 5 |
| 60 | 0.8 | 3 | [MAT](cdc2d_60kph_mu0.8_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.8_delta+3_trim_phi00.png) | 168, 177, 272, 149, 216 | 1, 2, 3, 4, 5 |
| 60 | 0.7 | 0 | [MAT](cdc2d_60kph_mu0.7_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.7_delta+0_trim_phi00.png) | 93, 92, 92, 92, 90 | 없음 |
| 60 | 0.7 | 0.5 | [MAT](cdc2d_60kph_mu0.7_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.7_delta+0.5_trim_phi00.png) | 94, 92, 91, 92, 89 | 없음 |
| 60 | 0.7 | 1 | [MAT](cdc2d_60kph_mu0.7_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.7_delta+1_trim_phi00.png) | 98, 97, 96, 96, 93 | 없음 |
| 60 | 0.7 | 1.5 | [MAT](cdc2d_60kph_mu0.7_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.7_delta+1.5_trim_phi00.png) | 105, 102, 92, 104, 90 | 없음 |
| 60 | 0.7 | 2 | [MAT](cdc2d_60kph_mu0.7_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.7_delta+2_trim_phi00.png) | 119, 106, 98, 109, 106 | 5 |
| 60 | 0.7 | 2.5 | [MAT](cdc2d_60kph_mu0.7_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.7_delta+2.5_trim_phi00.png) | 115, 115, 116, 112, 128 | 5 |
| 60 | 0.7 | 3 | [MAT](cdc2d_60kph_mu0.7_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.7_delta+3_trim_phi00.png) | 123, 123, 151, 120, 151 | 3, 5 |
| 60 | 0.6 | 0 | [MAT](cdc2d_60kph_mu0.6_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.6_delta+0_trim_phi00.png) | 70, 70, 71, 70, 70 | 없음 |
| 60 | 0.6 | 0.5 | [MAT](cdc2d_60kph_mu0.6_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.6_delta+0.5_trim_phi00.png) | 72, 71, 66, 72, 68 | 없음 |
| 60 | 0.6 | 1 | [MAT](cdc2d_60kph_mu0.6_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.6_delta+1_trim_phi00.png) | 77, 72, 65, 71, 66 | 없음 |
| 60 | 0.6 | 1.5 | [MAT](cdc2d_60kph_mu0.6_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.6_delta+1.5_trim_phi00.png) | 72, 69, 66, 70, 65 | 없음 |
| 60 | 0.6 | 2 | [MAT](cdc2d_60kph_mu0.6_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.6_delta+2_trim_phi00.png) | 77, 74, 72, 76, 73 | 없음 |
| 60 | 0.6 | 2.5 | [MAT](cdc2d_60kph_mu0.6_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.6_delta+2.5_trim_phi00.png) | 91, 84, 81, 85, 91 | 5 |
| 60 | 0.6 | 3 | [MAT](cdc2d_60kph_mu0.6_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.6_delta+3_trim_phi00.png) | 32, 11, 0, 36, 37 | 없음 |
| 60 | 0.5 | 0 | [MAT](cdc2d_60kph_mu0.5_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.5_delta+0_trim_phi00.png) | 57, 53, 44, 53, 44 | 없음 |
| 60 | 0.5 | 0.5 | [MAT](cdc2d_60kph_mu0.5_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.5_delta+0.5_trim_phi00.png) | 54, 50, 47, 51, 48 | 없음 |
| 60 | 0.5 | 1 | [MAT](cdc2d_60kph_mu0.5_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.5_delta+1_trim_phi00.png) | 49, 47, 46, 49, 48 | 없음 |
| 60 | 0.5 | 1.5 | [MAT](cdc2d_60kph_mu0.5_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.5_delta+1.5_trim_phi00.png) | 51, 51, 50, 52, 51 | 없음 |
| 60 | 0.5 | 2 | [MAT](cdc2d_60kph_mu0.5_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.5_delta+2_trim_phi00.png) | 56, 55, 54, 56, 54 | 없음 |
| 60 | 0.5 | 2.5 | [MAT](cdc2d_60kph_mu0.5_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.5_delta+2.5_trim_phi00.png) | 17, 6, 0, 22, 22 | 없음 |
| 60 | 0.5 | 3 | [MAT](cdc2d_60kph_mu0.5_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_60kph_mu0.5_delta+3_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 70 | 0.9 | 0 | [MAT](cdc2d_70kph_mu0.9_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.9_delta+0_trim_phi00.png) | 145, 145, 139, 145, 123 | 없음 |
| 70 | 0.9 | 0.5 | [MAT](cdc2d_70kph_mu0.9_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.9_delta+0.5_trim_phi00.png) | 147, 145, 146, 148, 132 | 없음 |
| 70 | 0.9 | 1 | [MAT](cdc2d_70kph_mu0.9_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.9_delta+1_trim_phi00.png) | 148, 138, 148, 140, 137 | 5 |
| 70 | 0.9 | 1.5 | [MAT](cdc2d_70kph_mu0.9_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.9_delta+1.5_trim_phi00.png) | 145, 142, 160, 140, 152 | 3, 5 |
| 70 | 0.9 | 2 | [MAT](cdc2d_70kph_mu0.9_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.9_delta+2_trim_phi00.png) | 154, 153, 195, 150, 175 | 3, 5 |
| 70 | 0.9 | 2.5 | [MAT](cdc2d_70kph_mu0.9_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.9_delta+2.5_trim_phi00.png) | 176, 180, 259, 168, 199 | 3, 5 |
| 70 | 0.9 | 3 | [MAT](cdc2d_70kph_mu0.9_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.9_delta+3_trim_phi00.png) | 189, 204, 336, 179, 241 | 2, 3, 5 |
| 70 | 0.8 | 0 | [MAT](cdc2d_70kph_mu0.8_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.8_delta+0_trim_phi00.png) | 116, 105, 93, 106, 92 | 없음 |
| 70 | 0.8 | 0.5 | [MAT](cdc2d_70kph_mu0.8_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.8_delta+0.5_trim_phi00.png) | 114, 111, 93, 111, 92 | 없음 |
| 70 | 0.8 | 1 | [MAT](cdc2d_70kph_mu0.8_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.8_delta+1_trim_phi00.png) | 110, 108, 103, 110, 103 | 없음 |
| 70 | 0.8 | 1.5 | [MAT](cdc2d_70kph_mu0.8_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.8_delta+1.5_trim_phi00.png) | 114, 113, 113, 114, 112 | 없음 |
| 70 | 0.8 | 2 | [MAT](cdc2d_70kph_mu0.8_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.8_delta+2_trim_phi00.png) | 120, 118, 122, 119, 121 | 없음 |
| 70 | 0.8 | 2.5 | [MAT](cdc2d_70kph_mu0.8_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.8_delta+2.5_trim_phi00.png) | 132, 128, 137, 129, 142 | 3, 5 |
| 70 | 0.8 | 3 | [MAT](cdc2d_70kph_mu0.8_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.8_delta+3_trim_phi00.png) | 122, 0, 0, 124, 147 | 5 |
| 70 | 0.7 | 0 | [MAT](cdc2d_70kph_mu0.7_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.7_delta+0_trim_phi00.png) | 83, 80, 75, 80, 75 | 없음 |
| 70 | 0.7 | 0.5 | [MAT](cdc2d_70kph_mu0.7_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.7_delta+0.5_trim_phi00.png) | 83, 81, 79, 80, 77 | 없음 |
| 70 | 0.7 | 1 | [MAT](cdc2d_70kph_mu0.7_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.7_delta+1_trim_phi00.png) | 97, 91, 85, 88, 82 | 없음 |
| 70 | 0.7 | 1.5 | [MAT](cdc2d_70kph_mu0.7_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.7_delta+1.5_trim_phi00.png) | 100, 99, 93, 100, 93 | 없음 |
| 70 | 0.7 | 2 | [MAT](cdc2d_70kph_mu0.7_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.7_delta+2_trim_phi00.png) | 106, 104, 98, 105, 94 | 없음 |
| 70 | 0.7 | 2.5 | [MAT](cdc2d_70kph_mu0.7_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.7_delta+2.5_trim_phi00.png) | 112, 110, 0, 109, 109 | 없음 |
| 70 | 0.7 | 3 | [MAT](cdc2d_70kph_mu0.7_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.7_delta+3_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 70 | 0.6 | 0 | [MAT](cdc2d_70kph_mu0.6_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.6_delta+0_trim_phi00.png) | 57, 57, 57, 57, 57 | 없음 |
| 70 | 0.6 | 0.5 | [MAT](cdc2d_70kph_mu0.6_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.6_delta+0.5_trim_phi00.png) | 59, 58, 57, 60, 60 | 없음 |
| 70 | 0.6 | 1 | [MAT](cdc2d_70kph_mu0.6_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.6_delta+1_trim_phi00.png) | 65, 63, 57, 64, 60 | 없음 |
| 70 | 0.6 | 1.5 | [MAT](cdc2d_70kph_mu0.6_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.6_delta+1.5_trim_phi00.png) | 73, 70, 60, 71, 60 | 없음 |
| 70 | 0.6 | 2 | [MAT](cdc2d_70kph_mu0.6_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.6_delta+2_trim_phi00.png) | 79, 75, 68, 76, 69 | 없음 |
| 70 | 0.6 | 2.5 | [MAT](cdc2d_70kph_mu0.6_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.6_delta+2.5_trim_phi00.png) | 0, 0, 0, 0, 4 | 없음 |
| 70 | 0.6 | 3 | [MAT](cdc2d_70kph_mu0.6_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.6_delta+3_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 70 | 0.5 | 0 | [MAT](cdc2d_70kph_mu0.5_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.5_delta+0_trim_phi00.png) | 51, 51, 50, 51, 50 | 없음 |
| 70 | 0.5 | 0.5 | [MAT](cdc2d_70kph_mu0.5_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.5_delta+0.5_trim_phi00.png) | 51, 51, 44, 51, 46 | 없음 |
| 70 | 0.5 | 1 | [MAT](cdc2d_70kph_mu0.5_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.5_delta+1_trim_phi00.png) | 51, 45, 43, 48, 45 | 없음 |
| 70 | 0.5 | 1.5 | [MAT](cdc2d_70kph_mu0.5_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.5_delta+1.5_trim_phi00.png) | 52, 49, 48, 50, 49 | 없음 |
| 70 | 0.5 | 2 | [MAT](cdc2d_70kph_mu0.5_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.5_delta+2_trim_phi00.png) | 0, 0, 0, 3, 10 | 없음 |
| 70 | 0.5 | 2.5 | [MAT](cdc2d_70kph_mu0.5_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.5_delta+2.5_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 70 | 0.5 | 3 | [MAT](cdc2d_70kph_mu0.5_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_70kph_mu0.5_delta+3_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 80 | 0.9 | 0 | [MAT](cdc2d_80kph_mu0.9_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.9_delta+0_trim_phi00.png) | 127, 126, 128, 126, 125 | 없음 |
| 80 | 0.9 | 0.5 | [MAT](cdc2d_80kph_mu0.9_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.9_delta+0.5_trim_phi00.png) | 130, 128, 129, 129, 129 | 없음 |
| 80 | 0.9 | 1 | [MAT](cdc2d_80kph_mu0.9_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.9_delta+1_trim_phi00.png) | 140, 134, 122, 135, 120 | 없음 |
| 80 | 0.9 | 1.5 | [MAT](cdc2d_80kph_mu0.9_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.9_delta+1.5_trim_phi00.png) | 154, 153, 135, 153, 126 | 없음 |
| 80 | 0.9 | 2 | [MAT](cdc2d_80kph_mu0.9_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.9_delta+2_trim_phi00.png) | 160, 153, 159, 150, 149 | 5 |
| 80 | 0.9 | 2.5 | [MAT](cdc2d_80kph_mu0.9_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.9_delta+2.5_trim_phi00.png) | 165, 164, 0, 159, 170 | 5 |
| 80 | 0.9 | 3 | [MAT](cdc2d_80kph_mu0.9_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.9_delta+3_trim_phi00.png) | 0, 0, 0, 0, 11 | 없음 |
| 80 | 0.8 | 0 | [MAT](cdc2d_80kph_mu0.8_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.8_delta+0_trim_phi00.png) | 115, 115, 96, 115, 98 | 없음 |
| 80 | 0.8 | 0.5 | [MAT](cdc2d_80kph_mu0.8_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.8_delta+0.5_trim_phi00.png) | 115, 115, 100, 116, 101 | 없음 |
| 80 | 0.8 | 1 | [MAT](cdc2d_80kph_mu0.8_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.8_delta+1_trim_phi00.png) | 120, 107, 100, 108, 102 | 없음 |
| 80 | 0.8 | 1.5 | [MAT](cdc2d_80kph_mu0.8_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.8_delta+1.5_trim_phi00.png) | 115, 112, 108, 112, 108 | 없음 |
| 80 | 0.8 | 2 | [MAT](cdc2d_80kph_mu0.8_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.8_delta+2_trim_phi00.png) | 132, 124, 119, 124, 118 | 없음 |
| 80 | 0.8 | 2.5 | [MAT](cdc2d_80kph_mu0.8_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.8_delta+2.5_trim_phi00.png) | 0, 0, 0, 32, 53 | 없음 |
| 80 | 0.8 | 3 | [MAT](cdc2d_80kph_mu0.8_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.8_delta+3_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 80 | 0.7 | 0 | [MAT](cdc2d_80kph_mu0.7_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.7_delta+0_trim_phi00.png) | 84, 75, 63, 76, 63 | 없음 |
| 80 | 0.7 | 0.5 | [MAT](cdc2d_80kph_mu0.7_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.7_delta+0.5_trim_phi00.png) | 83, 81, 67, 80, 65 | 없음 |
| 80 | 0.7 | 1 | [MAT](cdc2d_80kph_mu0.7_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.7_delta+1_trim_phi00.png) | 82, 81, 80, 81, 79 | 없음 |
| 80 | 0.7 | 1.5 | [MAT](cdc2d_80kph_mu0.7_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.7_delta+1.5_trim_phi00.png) | 87, 85, 83, 86, 83 | 없음 |
| 80 | 0.7 | 2 | [MAT](cdc2d_80kph_mu0.7_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.7_delta+2_trim_phi00.png) | 94, 0, 0, 92, 90 | 없음 |
| 80 | 0.7 | 2.5 | [MAT](cdc2d_80kph_mu0.7_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.7_delta+2.5_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 80 | 0.7 | 3 | [MAT](cdc2d_80kph_mu0.7_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.7_delta+3_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 80 | 0.6 | 0 | [MAT](cdc2d_80kph_mu0.6_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.6_delta+0_trim_phi00.png) | 53, 52, 51, 52, 51 | 없음 |
| 80 | 0.6 | 0.5 | [MAT](cdc2d_80kph_mu0.6_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.6_delta+0.5_trim_phi00.png) | 56, 55, 52, 55, 55 | 없음 |
| 80 | 0.6 | 1 | [MAT](cdc2d_80kph_mu0.6_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.6_delta+1_trim_phi00.png) | 70, 65, 59, 62, 60 | 없음 |
| 80 | 0.6 | 1.5 | [MAT](cdc2d_80kph_mu0.6_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.6_delta+1.5_trim_phi00.png) | 73, 71, 70, 72, 67 | 없음 |
| 80 | 0.6 | 2 | [MAT](cdc2d_80kph_mu0.6_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.6_delta+2_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 80 | 0.6 | 2.5 | [MAT](cdc2d_80kph_mu0.6_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.6_delta+2.5_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 80 | 0.6 | 3 | [MAT](cdc2d_80kph_mu0.6_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.6_delta+3_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 80 | 0.5 | 0 | [MAT](cdc2d_80kph_mu0.5_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.5_delta+0_trim_phi00.png) | 45, 45, 45, 45, 45 | 없음 |
| 80 | 0.5 | 0.5 | [MAT](cdc2d_80kph_mu0.5_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.5_delta+0.5_trim_phi00.png) | 45, 45, 45, 46, 46 | 없음 |
| 80 | 0.5 | 1 | [MAT](cdc2d_80kph_mu0.5_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.5_delta+1_trim_phi00.png) | 50, 49, 41, 51, 43 | 없음 |
| 80 | 0.5 | 1.5 | [MAT](cdc2d_80kph_mu0.5_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.5_delta+1.5_trim_phi00.png) | 0, 0, 0, 25, 44 | 없음 |
| 80 | 0.5 | 2 | [MAT](cdc2d_80kph_mu0.5_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.5_delta+2_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 80 | 0.5 | 2.5 | [MAT](cdc2d_80kph_mu0.5_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.5_delta+2.5_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 80 | 0.5 | 3 | [MAT](cdc2d_80kph_mu0.5_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_80kph_mu0.5_delta+3_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 90 | 0.9 | 0 | [MAT](cdc2d_90kph_mu0.9_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.9_delta+0_trim_phi00.png) | 135, 123, 116, 123, 117 | 없음 |
| 90 | 0.9 | 0.5 | [MAT](cdc2d_90kph_mu0.9_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.9_delta+0.5_trim_phi00.png) | 143, 127, 117, 127, 121 | 없음 |
| 90 | 0.9 | 1 | [MAT](cdc2d_90kph_mu0.9_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.9_delta+1_trim_phi00.png) | 143, 142, 128, 140, 128 | 없음 |
| 90 | 0.9 | 1.5 | [MAT](cdc2d_90kph_mu0.9_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.9_delta+1.5_trim_phi00.png) | 149, 146, 148, 147, 138 | 없음 |
| 90 | 0.9 | 2 | [MAT](cdc2d_90kph_mu0.9_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.9_delta+2_trim_phi00.png) | 159, 156, 0, 154, 141 | 없음 |
| 90 | 0.9 | 2.5 | [MAT](cdc2d_90kph_mu0.9_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.9_delta+2.5_trim_phi00.png) | 0, 0, 0, 0, 6 | 없음 |
| 90 | 0.9 | 3 | [MAT](cdc2d_90kph_mu0.9_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.9_delta+3_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 90 | 0.8 | 0 | [MAT](cdc2d_90kph_mu0.8_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.8_delta+0_trim_phi00.png) | 105, 105, 105, 105, 105 | 없음 |
| 90 | 0.8 | 0.5 | [MAT](cdc2d_90kph_mu0.8_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.8_delta+0.5_trim_phi00.png) | 108, 105, 105, 108, 105 | 없음 |
| 90 | 0.8 | 1 | [MAT](cdc2d_90kph_mu0.8_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.8_delta+1_trim_phi00.png) | 118, 112, 99, 114, 99 | 없음 |
| 90 | 0.8 | 1.5 | [MAT](cdc2d_90kph_mu0.8_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.8_delta+1.5_trim_phi00.png) | 129, 128, 110, 128, 108 | 없음 |
| 90 | 0.8 | 2 | [MAT](cdc2d_90kph_mu0.8_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.8_delta+2_trim_phi00.png) | 0, 0, 0, 51, 73 | 없음 |
| 90 | 0.8 | 2.5 | [MAT](cdc2d_90kph_mu0.8_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.8_delta+2.5_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 90 | 0.8 | 3 | [MAT](cdc2d_90kph_mu0.8_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.8_delta+3_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 90 | 0.7 | 0 | [MAT](cdc2d_90kph_mu0.7_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.7_delta+0_trim_phi00.png) | 85, 85, 64, 85, 64 | 없음 |
| 90 | 0.7 | 0.5 | [MAT](cdc2d_90kph_mu0.7_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.7_delta+0.5_trim_phi00.png) | 85, 85, 73, 85, 70 | 없음 |
| 90 | 0.7 | 1 | [MAT](cdc2d_90kph_mu0.7_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.7_delta+1_trim_phi00.png) | 81, 77, 74, 78, 73 | 없음 |
| 90 | 0.7 | 1.5 | [MAT](cdc2d_90kph_mu0.7_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.7_delta+1.5_trim_phi00.png) | 87, 84, 0, 85, 80 | 없음 |
| 90 | 0.7 | 2 | [MAT](cdc2d_90kph_mu0.7_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.7_delta+2_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 90 | 0.7 | 2.5 | [MAT](cdc2d_90kph_mu0.7_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.7_delta+2.5_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 90 | 0.7 | 3 | [MAT](cdc2d_90kph_mu0.7_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.7_delta+3_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 90 | 0.6 | 0 | [MAT](cdc2d_90kph_mu0.6_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.6_delta+0_trim_phi00.png) | 62, 50, 46, 50, 46 | 없음 |
| 90 | 0.6 | 0.5 | [MAT](cdc2d_90kph_mu0.6_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.6_delta+0.5_trim_phi00.png) | 63, 57, 48, 55, 50 | 없음 |
| 90 | 0.6 | 1 | [MAT](cdc2d_90kph_mu0.6_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.6_delta+1_trim_phi00.png) | 63, 63, 63, 62, 61 | 없음 |
| 90 | 0.6 | 1.5 | [MAT](cdc2d_90kph_mu0.6_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.6_delta+1.5_trim_phi00.png) | 0, 0, 0, 21, 41 | 없음 |
| 90 | 0.6 | 2 | [MAT](cdc2d_90kph_mu0.6_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.6_delta+2_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 90 | 0.6 | 2.5 | [MAT](cdc2d_90kph_mu0.6_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.6_delta+2.5_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 90 | 0.6 | 3 | [MAT](cdc2d_90kph_mu0.6_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.6_delta+3_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 90 | 0.5 | 0 | [MAT](cdc2d_90kph_mu0.5_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.5_delta+0_trim_phi00.png) | 39, 39, 39, 39, 39 | 없음 |
| 90 | 0.5 | 0.5 | [MAT](cdc2d_90kph_mu0.5_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.5_delta+0.5_trim_phi00.png) | 40, 39, 39, 41, 41 | 없음 |
| 90 | 0.5 | 1 | [MAT](cdc2d_90kph_mu0.5_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.5_delta+1_trim_phi00.png) | 49, 46, 0, 47, 44 | 없음 |
| 90 | 0.5 | 1.5 | [MAT](cdc2d_90kph_mu0.5_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.5_delta+1.5_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 90 | 0.5 | 2 | [MAT](cdc2d_90kph_mu0.5_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.5_delta+2_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 90 | 0.5 | 2.5 | [MAT](cdc2d_90kph_mu0.5_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.5_delta+2.5_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 90 | 0.5 | 3 | [MAT](cdc2d_90kph_mu0.5_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_90kph_mu0.5_delta+3_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 100 | 0.9 | 0 | [MAT](cdc2d_100kph_mu0.9_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_100kph_mu0.9_delta+0_trim_phi00.png) | 147, 138, 110, 138, 110 | 없음 |
| 100 | 0.9 | 0.5 | [MAT](cdc2d_100kph_mu0.9_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_100kph_mu0.9_delta+0.5_trim_phi00.png) | 147, 137, 117, 136, 115 | 없음 |
| 100 | 0.9 | 1 | [MAT](cdc2d_100kph_mu0.9_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_100kph_mu0.9_delta+1_trim_phi00.png) | 141, 135, 132, 135, 129 | 없음 |
| 100 | 0.9 | 1.5 | [MAT](cdc2d_100kph_mu0.9_delta+1.5_trim_phi00.mat) | [그림](figures/cdc2d_100kph_mu0.9_delta+1.5_trim_phi00.png) | 154, 147, 0, 144, 135 | 없음 |
| 100 | 0.9 | 2 | [MAT](cdc2d_100kph_mu0.9_delta+2_trim_phi00.mat) | [그림](figures/cdc2d_100kph_mu0.9_delta+2_trim_phi00.png) | 0, 0, 0, 4, 40 | 없음 |
| 100 | 0.9 | 2.5 | [MAT](cdc2d_100kph_mu0.9_delta+2.5_trim_phi00.mat) | [그림](figures/cdc2d_100kph_mu0.9_delta+2.5_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 100 | 0.9 | 3 | [MAT](cdc2d_100kph_mu0.9_delta+3_trim_phi00.mat) | [그림](figures/cdc2d_100kph_mu0.9_delta+3_trim_phi00.png) | 0, 0, 0, 0, 0 | 없음 |
| 100 | 0.8 | 0 | [MAT](cdc2d_100kph_mu0.8_delta+0_trim_phi00.mat) | [그림](figures/cdc2d_100kph_mu0.8_delta+0_trim_phi00.png) | 105, 96, 95, 97, 95 | 없음 |
| 100 | 0.8 | 0.5 | [MAT](cdc2d_100kph_mu0.8_delta+0.5_trim_phi00.mat) | [그림](figures/cdc2d_100kph_mu0.8_delta+0.5_trim_phi00.png) | 116, 101, 95, 102, 99 | 없음 |
| 100 | 0.8 | 1 | [MAT](cdc2d_100kph_mu0.8_delta+1_trim_phi00.mat) | [그림](figures/cdc2d_100kph_mu0.8_delta+1_trim_phi00.png) | 117, 117, 104, 114, 106 | 없음 |
| 100 | 0.8 | 1.5 | [MAT](cdc2d_100kph_mu0.8_delta+1.5_trim_phi00.mat) | 생성 중 | 121, 0, 0, 117, 107 | 없음 |

## 조향각 7종 비교

- [50 km/h · μ=0.9](figures/overview_selected_50kph_mu0p9_trim.png)
- [50 km/h · μ=0.8](figures/overview_selected_50kph_mu0p8_trim.png)
- [50 km/h · μ=0.7](figures/overview_selected_50kph_mu0p7_trim.png)
- [50 km/h · μ=0.6](figures/overview_selected_50kph_mu0p6_trim.png)
- [50 km/h · μ=0.5](figures/overview_selected_50kph_mu0p5_trim.png)
- [60 km/h · μ=0.9](figures/overview_selected_60kph_mu0p9_trim.png)
- [60 km/h · μ=0.8](figures/overview_selected_60kph_mu0p8_trim.png)
- [60 km/h · μ=0.7](figures/overview_selected_60kph_mu0p7_trim.png)
- [60 km/h · μ=0.6](figures/overview_selected_60kph_mu0p6_trim.png)
- [60 km/h · μ=0.5](figures/overview_selected_60kph_mu0p5_trim.png)
- [70 km/h · μ=0.9](figures/overview_selected_70kph_mu0p9_trim.png)
- [70 km/h · μ=0.8](figures/overview_selected_70kph_mu0p8_trim.png)
- [70 km/h · μ=0.7](figures/overview_selected_70kph_mu0p7_trim.png)
- [70 km/h · μ=0.6](figures/overview_selected_70kph_mu0p6_trim.png)
- [70 km/h · μ=0.5](figures/overview_selected_70kph_mu0p5_trim.png)
- [80 km/h · μ=0.9](figures/overview_selected_80kph_mu0p9_trim.png)
- [80 km/h · μ=0.8](figures/overview_selected_80kph_mu0p8_trim.png)
- [80 km/h · μ=0.7](figures/overview_selected_80kph_mu0p7_trim.png)
- [80 km/h · μ=0.6](figures/overview_selected_80kph_mu0p6_trim.png)
- [80 km/h · μ=0.5](figures/overview_selected_80kph_mu0p5_trim.png)
- [90 km/h · μ=0.9](figures/overview_selected_90kph_mu0p9_trim.png)
- [90 km/h · μ=0.8](figures/overview_selected_90kph_mu0p8_trim.png)
- [90 km/h · μ=0.7](figures/overview_selected_90kph_mu0p7_trim.png)
- [90 km/h · μ=0.6](figures/overview_selected_90kph_mu0p6_trim.png)
- [90 km/h · μ=0.5](figures/overview_selected_90kph_mu0p5_trim.png)
- [100 km/h · μ=0.9](figures/overview_selected_100kph_mu0p9_trim.png)
