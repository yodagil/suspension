# CDC Research Final: H-preserving recovery

**검증 후 업로드: 12/210조건 · 조건별 마모 5종.** 계산 상태: `running`.

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

## 조향각 7종 비교

- [50 km/h · μ=0.9](figures/overview_final_50kph_mu0p9_trim.png)
