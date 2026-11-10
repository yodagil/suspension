# V2 Full-car 2차원 CDC 안전·복귀 영역

**검증 후 업로드: 25/210조건 · 조건별 마모 5종.** 계산 상태: `running`.

[현재 진행 상황](publication_progress.json) · [전체 수치 요약](publication_summary.csv) · [실행·판정 설명](../../fullcar_2d_v2/README_KO.md)

초기조건은 횡속도 49개 × 요레이트 81개입니다. **V2 기본 trim 단면**을 사용해 롤각을 포함한 숨은 상태를 각 마모의 계산된 정상선회점에 맞추고, 32상태 Full-car 모델의 롤·현가 운동을 계속 적분합니다. 그래프의 두 축은 초기 횡속도와 요레이트입니다.

첨부 V2 Full-car 코드의 trim 초기화와 5초·10초 판정을 유지합니다. `safe`, `guarded_safe`, `recoverable`, `guarded_recoverable`, `safe_unresolved`를 따로 저장하며 대표 선택 영역은 **10초 recoverable**입니다. 절대 요레이트 정책은 별도 마스크이며 대표 영역에 자동으로 교차시키지 않습니다. 유한시간 표본 판정이며 증명된 점근 안정영역이나 불변집합을 뜻하지 않습니다.

MAT 변수명은 `STABILITY2D`입니다. `stable_points`는 초기 `[vc0, r0]` 좌표이고, `region.sampled_area`는 네 꼭짓점과 요청된 중심점 검사를 통과한 2차원 셀 면적입니다. 격자 끝에 닿은 결과는 잘렸을 수 있으므로 아래 표시와 원시 마스크를 함께 확인하세요.

| 속도 km/h | μ | 앞 휠 조향각 ° | MAT | 2D 그림 | 선택점 수, 마모 1~5 | 격자 경계 접촉 |
|---|---|---|---|---|---|---|
| 50 | 0.9 | 0 | [MAT](fullcar2d_v2_50kph_mu0.9_delta+0_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.9_delta+0_trim_trim.png) | 1519, 1571, 1818, 1460, 1629 | 1, 2, 3, 4, 5 |
| 50 | 0.9 | 0.5 | [MAT](fullcar2d_v2_50kph_mu0.9_delta+0.5_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.9_delta+0.5_trim_trim.png) | 1523, 1578, 1832, 1466, 1634 | 1, 2, 3, 4, 5 |
| 50 | 0.9 | 1 | [MAT](fullcar2d_v2_50kph_mu0.9_delta+1_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.9_delta+1_trim_trim.png) | 1529, 1587, 1862, 1474, 1651 | 1, 2, 3, 4, 5 |
| 50 | 0.9 | 1.5 | [MAT](fullcar2d_v2_50kph_mu0.9_delta+1.5_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.9_delta+1.5_trim_trim.png) | 1553, 1602, 1872, 1487, 1664 | 1, 2, 3, 4, 5 |
| 50 | 0.9 | 2 | [MAT](fullcar2d_v2_50kph_mu0.9_delta+2_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.9_delta+2_trim_trim.png) | 1585, 1622, 1856, 1520, 1687 | 1, 2, 3, 4, 5 |
| 50 | 0.9 | 2.5 | [MAT](fullcar2d_v2_50kph_mu0.9_delta+2.5_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.9_delta+2.5_trim_trim.png) | 1597, 1649, 1859, 1560, 1720 | 1, 2, 3, 4, 5 |
| 50 | 0.9 | 3 | [MAT](fullcar2d_v2_50kph_mu0.9_delta+3_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.9_delta+3_trim_trim.png) | 1632, 1657, 1873, 1574, 1763 | 1, 2, 3, 4, 5 |
| 50 | 0.8 | 0 | [MAT](fullcar2d_v2_50kph_mu0.8_delta+0_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.8_delta+0_trim_trim.png) | 1429, 1486, 1703, 1372, 1492 | 1, 2, 3, 4, 5 |
| 50 | 0.8 | 0.5 | [MAT](fullcar2d_v2_50kph_mu0.8_delta+0.5_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.8_delta+0.5_trim_trim.png) | 1443, 1496, 1672, 1380, 1504 | 1, 2, 3, 4, 5 |
| 50 | 0.8 | 1 | [MAT](fullcar2d_v2_50kph_mu0.8_delta+1_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.8_delta+1_trim_trim.png) | 1453, 1498, 1645, 1387, 1520 | 1, 2, 3, 4, 5 |
| 50 | 0.8 | 1.5 | [MAT](fullcar2d_v2_50kph_mu0.8_delta+1.5_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.8_delta+1.5_trim_trim.png) | 1464, 1490, 1624, 1414, 1548 | 1, 2, 3, 4, 5 |
| 50 | 0.8 | 2 | [MAT](fullcar2d_v2_50kph_mu0.8_delta+2_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.8_delta+2_trim_trim.png) | 1466, 1464, 1604, 1416, 1569 | 1, 2, 3, 4, 5 |
| 50 | 0.8 | 2.5 | [MAT](fullcar2d_v2_50kph_mu0.8_delta+2.5_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.8_delta+2.5_trim_trim.png) | 1469, 1466, 1603, 1390, 1586 | 1, 2, 3, 4, 5 |
| 50 | 0.8 | 3 | [MAT](fullcar2d_v2_50kph_mu0.8_delta+3_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.8_delta+3_trim_trim.png) | 1464, 1467, 1606, 1400, 1546 | 1, 2, 3, 4, 5 |
| 50 | 0.7 | 0 | [MAT](fullcar2d_v2_50kph_mu0.7_delta+0_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.7_delta+0_trim_trim.png) | 1328, 1305, 1396, 1299, 1390 | 1, 2, 3, 4, 5 |
| 50 | 0.7 | 0.5 | [MAT](fullcar2d_v2_50kph_mu0.7_delta+0.5_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.7_delta+0.5_trim_trim.png) | 1348, 1310, 1396, 1299, 1398 | 1, 2, 3, 4, 5 |
| 50 | 0.7 | 1 | [MAT](fullcar2d_v2_50kph_mu0.7_delta+1_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.7_delta+1_trim_trim.png) | 1299, 1292, 1384, 1270, 1408 | 1, 2, 3, 4, 5 |
| 50 | 0.7 | 1.5 | [MAT](fullcar2d_v2_50kph_mu0.7_delta+1.5_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.7_delta+1.5_trim_trim.png) | 1285, 1280, 1361, 1224, 1368 | 1, 2, 3, 4, 5 |
| 50 | 0.7 | 2 | [MAT](fullcar2d_v2_50kph_mu0.7_delta+2_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.7_delta+2_trim_trim.png) | 1263, 1265, 1356, 1199, 1301 | 1, 2, 3, 4, 5 |
| 50 | 0.7 | 2.5 | [MAT](fullcar2d_v2_50kph_mu0.7_delta+2.5_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.7_delta+2.5_trim_trim.png) | 1256, 1254, 1324, 1189, 1269 | 1, 2, 3, 4, 5 |
| 50 | 0.7 | 3 | [MAT](fullcar2d_v2_50kph_mu0.7_delta+3_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.7_delta+3_trim_trim.png) | 1278, 1272, 1330, 1206, 1261 | 1, 2, 3, 4, 5 |
| 50 | 0.6 | 0 | [MAT](fullcar2d_v2_50kph_mu0.6_delta+0_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.6_delta+0_trim_trim.png) | 1029, 970, 1008, 1057, 1151 | 1, 2, 3, 4, 5 |
| 50 | 0.6 | 0.5 | [MAT](fullcar2d_v2_50kph_mu0.6_delta+0.5_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.6_delta+0.5_trim_trim.png) | 1028, 965, 1022, 1039, 1107 | 1, 2, 3, 4, 5 |
| 50 | 0.6 | 1 | [MAT](fullcar2d_v2_50kph_mu0.6_delta+1_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.6_delta+1_trim_trim.png) | 1028, 982, 1050, 994, 1055 | 1, 2, 3, 4, 5 |
| 50 | 0.6 | 1.5 | [MAT](fullcar2d_v2_50kph_mu0.6_delta+1.5_trim_trim.mat) | [그림](figures/fullcar2d_v2_50kph_mu0.6_delta+1.5_trim_trim.png) | 1052, 1013, 1082, 990, 1009 | 1, 2, 3, 4, 5 |

## 조향각 7종 비교

- [50 km/h · μ=0.9](figures/overview_selected_50kph_mu0p9_trim_trim.png)
- [50 km/h · μ=0.8](figures/overview_selected_50kph_mu0p8_trim_trim.png)
- [50 km/h · μ=0.7](figures/overview_selected_50kph_mu0p7_trim_trim.png)
