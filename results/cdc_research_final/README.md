# CDC Research Final: H-preserving recovery

**검증 후 업로드: 133/210조건 · 조건별 마모 5종.** 계산 상태: `running`.

[현재 진행 상황](publication_progress.json) · [전체 수치 요약](publication_summary.csv) · [실행·판정 설명](../../cdc_research_final/README_KO.md)

첨부 최종 연구 코드의 32상태 Full-car·MF 타이어·CDC 모델을 사용합니다. 초기조건은 횡속도 −3~3 m/s, 요레이트 −1.5~1.5 rad/s의 **61×121 격자**입니다. 각 마모의 정상선회점으로 롤각과 숨은 상태를 초기화하고 횡속도·요레이트만 바꿉니다.

**주 결과는 `masks.final`입니다.** 정적 하중으로 계산한 handling 기준 H 안의 초기점만 적분하고, H를 한 번도 벗어나지 않으면서 경로 제약을 만족하고 정상선회점으로 복귀한 점을 저장합니다. H 밖은 **미계산**이며 불안정 판정이 아닙니다. `recoverable_on_reference`는 H 이탈을 허용한 복귀 비교, `guarded_final`은 추가 요레이트 정책, `prepeak_final`은 추가 타이어 복원기울기 조건입니다.

MAT 변수는 **`RESEARCH2D`**이며 `CASE`는 마모 5종의 cell 배열입니다. 5초·10초 판정과 원시 진단을 함께 저장합니다. `region.cell_area`는 10초 final의 네 꼭짓점과 실제 중심점 시뮬레이션이 통과한 셀 면적입니다. 볼록껍질이나 영역 채우기를 하지 않습니다. 유한시간 표본 결과이며 연속영역·점근 안정성·불변집합의 수학적 보증은 아닙니다.

| 속도 km/h | μ | 앞 휠 조향각 ° | MAT | 2D 그림 | 10초 final 점 수, 마모 1~5 | 격자 접촉 |
|---|---|---|---|---|---|---|
| 50 | 0.9 | 0 | [MAT](research_50kph_mu0.9_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.9_delta+0_I950_950_950_950_trim.png) | 635, 569, 428, 591, 451 | 없음 |
| 50 | 0.9 | 0.5 | [MAT](research_50kph_mu0.9_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.9_delta+0.5_I950_950_950_950_trim.png) | 637, 575, 434, 595, 454 | 없음 |
| 50 | 0.9 | 1 | [MAT](research_50kph_mu0.9_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.9_delta+1_I950_950_950_950_trim.png) | 643, 568, 428, 590, 450 | 없음 |
| 50 | 0.9 | 1.5 | [MAT](research_50kph_mu0.9_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.9_delta+1.5_I950_950_950_950_trim.png) | 636, 575, 432, 596, 454 | 없음 |
| 50 | 0.9 | 2 | [MAT](research_50kph_mu0.9_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.9_delta+2_I950_950_950_950_trim.png) | 639, 571, 433, 593, 455 | 없음 |
| 50 | 0.9 | 2.5 | [MAT](research_50kph_mu0.9_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.9_delta+2.5_I950_950_950_950_trim.png) | 638, 569, 429, 593, 452 | 없음 |
| 50 | 0.9 | 3 | [MAT](research_50kph_mu0.9_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.9_delta+3_I950_950_950_950_trim.png) | 637, 573, 431, 594, 449 | 없음 |
| 50 | 0.8 | 0 | [MAT](research_50kph_mu0.8_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.8_delta+0_I950_950_950_950_trim.png) | 504, 451, 342, 470, 360 | 없음 |
| 50 | 0.8 | 0.5 | [MAT](research_50kph_mu0.8_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.8_delta+0.5_I950_950_950_950_trim.png) | 505, 455, 340, 472, 358 | 없음 |
| 50 | 0.8 | 1 | [MAT](research_50kph_mu0.8_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.8_delta+1_I950_950_950_950_trim.png) | 510, 450, 340, 466, 356 | 없음 |
| 50 | 0.8 | 1.5 | [MAT](research_50kph_mu0.8_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.8_delta+1.5_I950_950_950_950_trim.png) | 500, 453, 342, 471, 357 | 없음 |
| 50 | 0.8 | 2 | [MAT](research_50kph_mu0.8_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.8_delta+2_I950_950_950_950_trim.png) | 505, 452, 342, 473, 358 | 없음 |
| 50 | 0.8 | 2.5 | [MAT](research_50kph_mu0.8_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.8_delta+2.5_I950_950_950_950_trim.png) | 508, 451, 342, 468, 358 | 없음 |
| 50 | 0.8 | 3 | [MAT](research_50kph_mu0.8_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.8_delta+3_I950_950_950_950_trim.png) | 499, 454, 338, 467, 357 | 없음 |
| 50 | 0.7 | 0 | [MAT](research_50kph_mu0.7_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.7_delta+0_I950_950_950_950_trim.png) | 386, 346, 264, 361, 276 | 없음 |
| 50 | 0.7 | 0.5 | [MAT](research_50kph_mu0.7_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.7_delta+0.5_I950_950_950_950_trim.png) | 387, 345, 258, 361, 272 | 없음 |
| 50 | 0.7 | 1 | [MAT](research_50kph_mu0.7_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.7_delta+1_I950_950_950_950_trim.png) | 392, 350, 264, 361, 277 | 없음 |
| 50 | 0.7 | 1.5 | [MAT](research_50kph_mu0.7_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.7_delta+1.5_I950_950_950_950_trim.png) | 382, 345, 260, 357, 273 | 없음 |
| 50 | 0.7 | 2 | [MAT](research_50kph_mu0.7_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.7_delta+2_I950_950_950_950_trim.png) | 387, 343, 260, 363, 272 | 없음 |
| 50 | 0.7 | 2.5 | [MAT](research_50kph_mu0.7_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.7_delta+2.5_I950_950_950_950_trim.png) | 388, 348, 262, 359, 276 | 없음 |
| 50 | 0.7 | 3 | [MAT](research_50kph_mu0.7_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.7_delta+3_I950_950_950_950_trim.png) | 383, 344, 258, 357, 271 | 없음 |
| 50 | 0.6 | 0 | [MAT](research_50kph_mu0.6_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.6_delta+0_I950_950_950_950_trim.png) | 287, 256, 190, 265, 202 | 없음 |
| 50 | 0.6 | 0.5 | [MAT](research_50kph_mu0.6_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.6_delta+0.5_I950_950_950_950_trim.png) | 284, 254, 194, 262, 204 | 없음 |
| 50 | 0.6 | 1 | [MAT](research_50kph_mu0.6_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.6_delta+1_I950_950_950_950_trim.png) | 286, 259, 193, 269, 205 | 없음 |
| 50 | 0.6 | 1.5 | [MAT](research_50kph_mu0.6_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.6_delta+1.5_I950_950_950_950_trim.png) | 284, 252, 192, 262, 201 | 없음 |
| 50 | 0.6 | 2 | [MAT](research_50kph_mu0.6_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.6_delta+2_I950_950_950_950_trim.png) | 284, 254, 191, 264, 201 | 없음 |
| 50 | 0.6 | 2.5 | [MAT](research_50kph_mu0.6_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.6_delta+2.5_I950_950_950_950_trim.png) | 282, 257, 190, 268, 200 | 없음 |
| 50 | 0.6 | 3 | [MAT](research_50kph_mu0.6_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.6_delta+3_I950_950_950_950_trim.png) | 284, 251, 194, 263, 200 | 없음 |
| 50 | 0.5 | 0 | [MAT](research_50kph_mu0.5_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.5_delta+0_I950_950_950_950_trim.png) | 200, 179, 132, 186, 138 | 없음 |
| 50 | 0.5 | 0.5 | [MAT](research_50kph_mu0.5_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.5_delta+0.5_I950_950_950_950_trim.png) | 199, 177, 136, 182, 143 | 없음 |
| 50 | 0.5 | 1 | [MAT](research_50kph_mu0.5_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.5_delta+1_I950_950_950_950_trim.png) | 197, 177, 132, 185, 137 | 없음 |
| 50 | 0.5 | 1.5 | [MAT](research_50kph_mu0.5_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.5_delta+1.5_I950_950_950_950_trim.png) | 197, 174, 136, 181, 143 | 없음 |
| 50 | 0.5 | 2 | [MAT](research_50kph_mu0.5_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.5_delta+2_I950_950_950_950_trim.png) | 198, 178, 133, 185, 141 | 없음 |
| 50 | 0.5 | 2.5 | [MAT](research_50kph_mu0.5_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.5_delta+2.5_I950_950_950_950_trim.png) | 195, 177, 132, 184, 137 | 없음 |
| 50 | 0.5 | 3 | [MAT](research_50kph_mu0.5_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_50kph_mu0.5_delta+3_I950_950_950_950_trim.png) | 198, 174, 132, 178, 137 | 없음 |
| 60 | 0.9 | 0 | [MAT](research_60kph_mu0.9_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.9_delta+0_I950_950_950_950_trim.png) | 842, 758, 573, 790, 611 | 없음 |
| 60 | 0.9 | 0.5 | [MAT](research_60kph_mu0.9_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.9_delta+0.5_I950_950_950_950_trim.png) | 848, 751, 575, 786, 612 | 없음 |
| 60 | 0.9 | 1 | [MAT](research_60kph_mu0.9_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.9_delta+1_I950_950_950_950_trim.png) | 841, 756, 575, 795, 611 | 없음 |
| 60 | 0.9 | 1.5 | [MAT](research_60kph_mu0.9_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.9_delta+1.5_I950_950_950_950_trim.png) | 838, 757, 581, 794, 615 | 없음 |
| 60 | 0.9 | 2 | [MAT](research_60kph_mu0.9_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.9_delta+2_I950_950_950_950_trim.png) | 842, 756, 574, 792, 612 | 없음 |
| 60 | 0.9 | 2.5 | [MAT](research_60kph_mu0.9_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.9_delta+2.5_I950_950_950_950_trim.png) | 838, 753, 578, 790, 614 | 없음 |
| 60 | 0.9 | 3 | [MAT](research_60kph_mu0.9_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.9_delta+3_I950_950_950_950_trim.png) | 844, 752, 573, 783, 610 | 없음 |
| 60 | 0.8 | 0 | [MAT](research_60kph_mu0.8_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.8_delta+0_I950_950_950_950_trim.png) | 668, 596, 456, 625, 487 | 없음 |
| 60 | 0.8 | 0.5 | [MAT](research_60kph_mu0.8_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.8_delta+0.5_I950_950_950_950_trim.png) | 670, 597, 451, 623, 484 | 없음 |
| 60 | 0.8 | 1 | [MAT](research_60kph_mu0.8_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.8_delta+1_I950_950_950_950_trim.png) | 665, 598, 456, 626, 483 | 없음 |
| 60 | 0.8 | 1.5 | [MAT](research_60kph_mu0.8_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.8_delta+1.5_I950_950_950_950_trim.png) | 664, 599, 457, 629, 485 | 없음 |
| 60 | 0.8 | 2 | [MAT](research_60kph_mu0.8_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.8_delta+2_I950_950_950_950_trim.png) | 664, 597, 455, 621, 484 | 없음 |
| 60 | 0.8 | 2.5 | [MAT](research_60kph_mu0.8_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.8_delta+2.5_I950_950_950_950_trim.png) | 662, 594, 456, 624, 480 | 없음 |
| 60 | 0.8 | 3 | [MAT](research_60kph_mu0.8_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.8_delta+3_I950_950_950_950_trim.png) | 657, 591, 451, 617, 480 | 없음 |
| 60 | 0.7 | 0 | [MAT](research_60kph_mu0.7_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.7_delta+0_I950_950_950_950_trim.png) | 512, 457, 348, 481, 375 | 없음 |
| 60 | 0.7 | 0.5 | [MAT](research_60kph_mu0.7_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.7_delta+0.5_I950_950_950_950_trim.png) | 514, 457, 346, 476, 371 | 없음 |
| 60 | 0.7 | 1 | [MAT](research_60kph_mu0.7_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.7_delta+1_I950_950_950_950_trim.png) | 511, 459, 351, 481, 371 | 없음 |
| 60 | 0.7 | 1.5 | [MAT](research_60kph_mu0.7_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.7_delta+1.5_I950_950_950_950_trim.png) | 509, 459, 349, 481, 372 | 없음 |
| 60 | 0.7 | 2 | [MAT](research_60kph_mu0.7_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.7_delta+2_I950_950_950_950_trim.png) | 510, 453, 349, 476, 371 | 없음 |
| 60 | 0.7 | 2.5 | [MAT](research_60kph_mu0.7_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.7_delta+2.5_I950_950_950_950_trim.png) | 506, 454, 350, 472, 372 | 없음 |
| 60 | 0.7 | 3 | [MAT](research_60kph_mu0.7_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.7_delta+3_I950_950_950_950_trim.png) | 498, 448, 341, 470, 364 | 없음 |
| 60 | 0.6 | 0 | [MAT](research_60kph_mu0.6_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.6_delta+0_I950_950_950_950_trim.png) | 377, 335, 256, 354, 273 | 없음 |
| 60 | 0.6 | 0.5 | [MAT](research_60kph_mu0.6_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.6_delta+0.5_I950_950_950_950_trim.png) | 378, 337, 257, 352, 272 | 없음 |
| 60 | 0.6 | 1 | [MAT](research_60kph_mu0.6_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.6_delta+1_I950_950_950_950_trim.png) | 375, 338, 256, 352, 273 | 없음 |
| 60 | 0.6 | 1.5 | [MAT](research_60kph_mu0.6_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.6_delta+1.5_I950_950_950_950_trim.png) | 373, 340, 258, 354, 275 | 없음 |
| 60 | 0.6 | 2 | [MAT](research_60kph_mu0.6_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.6_delta+2_I950_950_950_950_trim.png) | 371, 331, 252, 349, 271 | 없음 |
| 60 | 0.6 | 2.5 | [MAT](research_60kph_mu0.6_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.6_delta+2.5_I950_950_950_950_trim.png) | 368, 327, 258, 341, 271 | 없음 |
| 60 | 0.6 | 3 | [MAT](research_60kph_mu0.6_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.6_delta+3_I950_950_950_950_trim.png) | 357, 326, 248, 339, 260 | 없음 |
| 60 | 0.5 | 0 | [MAT](research_60kph_mu0.5_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.5_delta+0_I950_950_950_950_trim.png) | 264, 234, 178, 246, 190 | 없음 |
| 60 | 0.5 | 0.5 | [MAT](research_60kph_mu0.5_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.5_delta+0.5_I950_950_950_950_trim.png) | 260, 234, 179, 246, 191 | 없음 |
| 60 | 0.5 | 1 | [MAT](research_60kph_mu0.5_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.5_delta+1_I950_950_950_950_trim.png) | 263, 235, 179, 242, 189 | 없음 |
| 60 | 0.5 | 1.5 | [MAT](research_60kph_mu0.5_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.5_delta+1.5_I950_950_950_950_trim.png) | 259, 236, 175, 248, 187 | 없음 |
| 60 | 0.5 | 2 | [MAT](research_60kph_mu0.5_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.5_delta+2_I950_950_950_950_trim.png) | 254, 227, 175, 237, 185 | 없음 |
| 60 | 0.5 | 2.5 | [MAT](research_60kph_mu0.5_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.5_delta+2.5_I950_950_950_950_trim.png) | 251, 223, 176, 232, 185 | 없음 |
| 60 | 0.5 | 3 | [MAT](research_60kph_mu0.5_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_60kph_mu0.5_delta+3_I950_950_950_950_trim.png) | 222, 0, 0, 210, 165 | 없음 |
| 70 | 0.9 | 0 | [MAT](research_70kph_mu0.9_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.9_delta+0_I950_950_950_950_trim.png) | 1033, 933, 721, 984, 776 | 없음 |
| 70 | 0.9 | 0.5 | [MAT](research_70kph_mu0.9_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.9_delta+0.5_I950_950_950_950_trim.png) | 1032, 936, 724, 988, 777 | 없음 |
| 70 | 0.9 | 1 | [MAT](research_70kph_mu0.9_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.9_delta+1_I950_950_950_950_trim.png) | 1031, 934, 723, 981, 779 | 없음 |
| 70 | 0.9 | 1.5 | [MAT](research_70kph_mu0.9_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.9_delta+1.5_I950_950_950_950_trim.png) | 1029, 928, 717, 974, 770 | 없음 |
| 70 | 0.9 | 2 | [MAT](research_70kph_mu0.9_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.9_delta+2_I950_950_950_950_trim.png) | 1022, 926, 712, 974, 764 | 없음 |
| 70 | 0.9 | 2.5 | [MAT](research_70kph_mu0.9_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.9_delta+2.5_I950_950_950_950_trim.png) | 1018, 918, 713, 964, 759 | 없음 |
| 70 | 0.9 | 3 | [MAT](research_70kph_mu0.9_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.9_delta+3_I950_950_950_950_trim.png) | 998, 907, 707, 946, 744 | 없음 |
| 70 | 0.8 | 0 | [MAT](research_70kph_mu0.8_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.8_delta+0_I950_950_950_950_trim.png) | 819, 735, 572, 779, 616 | 없음 |
| 70 | 0.8 | 0.5 | [MAT](research_70kph_mu0.8_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.8_delta+0.5_I950_950_950_950_trim.png) | 818, 741, 569, 781, 612 | 없음 |
| 70 | 0.8 | 1 | [MAT](research_70kph_mu0.8_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.8_delta+1_I950_950_950_950_trim.png) | 815, 737, 569, 776, 605 | 없음 |
| 70 | 0.8 | 1.5 | [MAT](research_70kph_mu0.8_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.8_delta+1.5_I950_950_950_950_trim.png) | 813, 730, 566, 765, 608 | 없음 |
| 70 | 0.8 | 2 | [MAT](research_70kph_mu0.8_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.8_delta+2_I950_950_950_950_trim.png) | 807, 725, 560, 760, 601 | 없음 |
| 70 | 0.8 | 2.5 | [MAT](research_70kph_mu0.8_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.8_delta+2.5_I950_950_950_950_trim.png) | 793, 714, 557, 750, 588 | 없음 |
| 70 | 0.8 | 3 | [MAT](research_70kph_mu0.8_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.8_delta+3_I950_950_950_950_trim.png) | 779, 704, 549, 732, 577 | 없음 |
| 70 | 0.7 | 0 | [MAT](research_70kph_mu0.7_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.7_delta+0_I950_950_950_950_trim.png) | 630, 565, 438, 598, 472 | 없음 |
| 70 | 0.7 | 0.5 | [MAT](research_70kph_mu0.7_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.7_delta+0.5_I950_950_950_950_trim.png) | 629, 564, 433, 593, 469 | 없음 |
| 70 | 0.7 | 1 | [MAT](research_70kph_mu0.7_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.7_delta+1_I950_950_950_950_trim.png) | 628, 563, 433, 590, 466 | 없음 |
| 70 | 0.7 | 1.5 | [MAT](research_70kph_mu0.7_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.7_delta+1.5_I950_950_950_950_trim.png) | 621, 558, 433, 590, 464 | 없음 |
| 70 | 0.7 | 2 | [MAT](research_70kph_mu0.7_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.7_delta+2_I950_950_950_950_trim.png) | 615, 548, 426, 576, 455 | 없음 |
| 70 | 0.7 | 2.5 | [MAT](research_70kph_mu0.7_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.7_delta+2.5_I950_950_950_950_trim.png) | 598, 542, 421, 561, 441 | 없음 |
| 70 | 0.7 | 3 | [MAT](research_70kph_mu0.7_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.7_delta+3_I950_950_950_950_trim.png) | 579, 493, 0, 544, 423 | 없음 |
| 70 | 0.6 | 0 | [MAT](research_70kph_mu0.6_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.6_delta+0_I950_950_950_950_trim.png) | 463, 415, 324, 438, 346 | 없음 |
| 70 | 0.6 | 0.5 | [MAT](research_70kph_mu0.6_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.6_delta+0.5_I950_950_950_950_trim.png) | 462, 413, 318, 439, 344 | 없음 |
| 70 | 0.6 | 1 | [MAT](research_70kph_mu0.6_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.6_delta+1_I950_950_950_950_trim.png) | 462, 412, 317, 433, 341 | 없음 |
| 70 | 0.6 | 1.5 | [MAT](research_70kph_mu0.6_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.6_delta+1.5_I950_950_950_950_trim.png) | 450, 409, 319, 431, 337 | 없음 |
| 70 | 0.6 | 2 | [MAT](research_70kph_mu0.6_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.6_delta+2_I950_950_950_950_trim.png) | 438, 400, 308, 418, 329 | 없음 |
| 70 | 0.6 | 2.5 | [MAT](research_70kph_mu0.6_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.6_delta+2.5_I950_950_950_950_trim.png) | 428, 373, 0, 400, 314 | 없음 |
| 70 | 0.6 | 3 | [MAT](research_70kph_mu0.6_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.6_delta+3_I950_950_950_950_trim.png) | 0, 0, 0, 55, 191 | 없음 |
| 70 | 0.5 | 0 | [MAT](research_70kph_mu0.5_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.5_delta+0_I950_950_950_950_trim.png) | 320, 289, 225, 306, 245 | 없음 |
| 70 | 0.5 | 0.5 | [MAT](research_70kph_mu0.5_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.5_delta+0.5_I950_950_950_950_trim.png) | 321, 289, 221, 302, 239 | 없음 |
| 70 | 0.5 | 1 | [MAT](research_70kph_mu0.5_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.5_delta+1_I950_950_950_950_trim.png) | 315, 286, 219, 304, 236 | 없음 |
| 70 | 0.5 | 1.5 | [MAT](research_70kph_mu0.5_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.5_delta+1.5_I950_950_950_950_trim.png) | 309, 278, 217, 291, 230 | 없음 |
| 70 | 0.5 | 2 | [MAT](research_70kph_mu0.5_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.5_delta+2_I950_950_950_950_trim.png) | 298, 269, 200, 283, 221 | 없음 |
| 70 | 0.5 | 2.5 | [MAT](research_70kph_mu0.5_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.5_delta+2.5_I950_950_950_950_trim.png) | 0, 0, 0, 3, 95 | 없음 |
| 70 | 0.5 | 3 | [MAT](research_70kph_mu0.5_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_70kph_mu0.5_delta+3_I950_950_950_950_trim.png) | 0, 0, 0, 0, 0 | 없음 |
| 80 | 0.9 | 0 | [MAT](research_80kph_mu0.9_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.9_delta+0_I950_950_950_950_trim.png) | 1206, 1083, 858, 1143, 929 | 없음 |
| 80 | 0.9 | 0.5 | [MAT](research_80kph_mu0.9_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.9_delta+0.5_I950_950_950_950_trim.png) | 1199, 1084, 853, 1143, 928 | 없음 |
| 80 | 0.9 | 1 | [MAT](research_80kph_mu0.9_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.9_delta+1_I950_950_950_950_trim.png) | 1190, 1076, 850, 1136, 920 | 없음 |
| 80 | 0.9 | 1.5 | [MAT](research_80kph_mu0.9_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.9_delta+1.5_I950_950_950_950_trim.png) | 1180, 1078, 842, 1133, 910 | 없음 |
| 80 | 0.9 | 2 | [MAT](research_80kph_mu0.9_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.9_delta+2_I950_950_950_950_trim.png) | 1166, 1055, 836, 1110, 882 | 1, 4 |
| 80 | 0.9 | 2.5 | [MAT](research_80kph_mu0.9_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.9_delta+2.5_I950_950_950_950_trim.png) | 1140, 1040, 826, 1083, 863 | 1, 2, 4 |
| 80 | 0.9 | 3 | [MAT](research_80kph_mu0.9_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.9_delta+3_I950_950_950_950_trim.png) | 1101, 958, 723, 1032, 809 | 1, 2, 4 |
| 80 | 0.8 | 0 | [MAT](research_80kph_mu0.8_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.8_delta+0_I950_950_950_950_trim.png) | 950, 860, 679, 909, 732 | 없음 |
| 80 | 0.8 | 0.5 | [MAT](research_80kph_mu0.8_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.8_delta+0.5_I950_950_950_950_trim.png) | 947, 860, 674, 909, 728 | 없음 |
| 80 | 0.8 | 1 | [MAT](research_80kph_mu0.8_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.8_delta+1_I950_950_950_950_trim.png) | 947, 851, 671, 894, 729 | 없음 |
| 80 | 0.8 | 1.5 | [MAT](research_80kph_mu0.8_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.8_delta+1.5_I950_950_950_950_trim.png) | 932, 837, 665, 884, 714 | 없음 |
| 80 | 0.8 | 2 | [MAT](research_80kph_mu0.8_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.8_delta+2_I950_950_950_950_trim.png) | 910, 829, 653, 864, 689 | 없음 |
| 80 | 0.8 | 2.5 | [MAT](research_80kph_mu0.8_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.8_delta+2.5_I950_950_950_950_trim.png) | 876, 803, 619, 829, 657 | 없음 |
| 80 | 0.8 | 3 | [MAT](research_80kph_mu0.8_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.8_delta+3_I950_950_950_950_trim.png) | 310, 0, 0, 712, 597 | 1, 4 |
| 80 | 0.7 | 0 | [MAT](research_80kph_mu0.7_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.7_delta+0_I950_950_950_950_trim.png) | 730, 659, 521, 698, 563 | 없음 |
| 80 | 0.7 | 0.5 | [MAT](research_80kph_mu0.7_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.7_delta+0.5_I950_950_950_950_trim.png) | 723, 660, 516, 692, 561 | 없음 |
| 80 | 0.7 | 1 | [MAT](research_80kph_mu0.7_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.7_delta+1_I950_950_950_950_trim.png) | 716, 652, 512, 689, 551 | 없음 |
| 80 | 0.7 | 1.5 | [MAT](research_80kph_mu0.7_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.7_delta+1.5_I950_950_950_950_trim.png) | 700, 637, 508, 675, 541 | 없음 |
| 80 | 0.7 | 2 | [MAT](research_80kph_mu0.7_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.7_delta+2_I950_950_950_950_trim.png) | 687, 625, 492, 644, 515 | 없음 |
| 80 | 0.7 | 2.5 | [MAT](research_80kph_mu0.7_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.7_delta+2.5_I950_950_950_950_trim.png) | 557, 0, 0, 577, 468 | 없음 |
| 80 | 0.7 | 3 | [MAT](research_80kph_mu0.7_delta+3_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.7_delta+3_I950_950_950_950_trim.png) | 0, 0, 0, 0, 0 | 없음 |
| 80 | 0.6 | 0 | [MAT](research_80kph_mu0.6_delta+0_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.6_delta+0_I950_950_950_950_trim.png) | 537, 482, 383, 513, 413 | 없음 |
| 80 | 0.6 | 0.5 | [MAT](research_80kph_mu0.6_delta+0.5_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.6_delta+0.5_I950_950_950_950_trim.png) | 536, 480, 377, 507, 411 | 없음 |
| 80 | 0.6 | 1 | [MAT](research_80kph_mu0.6_delta+1_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.6_delta+1_I950_950_950_950_trim.png) | 526, 475, 373, 497, 403 | 없음 |
| 80 | 0.6 | 1.5 | [MAT](research_80kph_mu0.6_delta+1.5_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.6_delta+1.5_I950_950_950_950_trim.png) | 508, 462, 366, 483, 390 | 없음 |
| 80 | 0.6 | 2 | [MAT](research_80kph_mu0.6_delta+2_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.6_delta+2_I950_950_950_950_trim.png) | 460, 0, 0, 455, 358 | 없음 |
| 80 | 0.6 | 2.5 | [MAT](research_80kph_mu0.6_delta+2.5_I950_950_950_950_trim.mat) | [그림](figures/research_80kph_mu0.6_delta+2.5_I950_950_950_950_trim.png) | 0, 0, 0, 0, 0 | 없음 |
| 80 | 0.6 | 3 | [MAT](research_80kph_mu0.6_delta+3_I950_950_950_950_trim.mat) | 생성 중 | 0, 0, 0, 0, 0 | 없음 |

## 조향각 7종 비교

- [50 km/h · μ=0.9](figures/overview_final_50kph_mu0p9_trim.png)
- [50 km/h · μ=0.8](figures/overview_final_50kph_mu0p8_trim.png)
- [50 km/h · μ=0.7](figures/overview_final_50kph_mu0p7_trim.png)
- [50 km/h · μ=0.6](figures/overview_final_50kph_mu0p6_trim.png)
- [50 km/h · μ=0.5](figures/overview_final_50kph_mu0p5_trim.png)
- [60 km/h · μ=0.9](figures/overview_final_60kph_mu0p9_trim.png)
- [60 km/h · μ=0.8](figures/overview_final_60kph_mu0p8_trim.png)
- [60 km/h · μ=0.7](figures/overview_final_60kph_mu0p7_trim.png)
- [60 km/h · μ=0.6](figures/overview_final_60kph_mu0p6_trim.png)
- [60 km/h · μ=0.5](figures/overview_final_60kph_mu0p5_trim.png)
- [70 km/h · μ=0.9](figures/overview_final_70kph_mu0p9_trim.png)
- [70 km/h · μ=0.8](figures/overview_final_70kph_mu0p8_trim.png)
- [70 km/h · μ=0.7](figures/overview_final_70kph_mu0p7_trim.png)
- [70 km/h · μ=0.6](figures/overview_final_70kph_mu0p6_trim.png)
- [70 km/h · μ=0.5](figures/overview_final_70kph_mu0p5_trim.png)
- [80 km/h · μ=0.9](figures/overview_final_80kph_mu0p9_trim.png)
- [80 km/h · μ=0.8](figures/overview_final_80kph_mu0p8_trim.png)
- [80 km/h · μ=0.7](figures/overview_final_80kph_mu0p7_trim.png)
