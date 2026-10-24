# 논문 수식 대응과 수치 재구성

| 논문 항목 | 구현 | 구분 |
|---|---|---|
| Table 1 | paper_config.m | 공개값 그대로 |
| Eq.1–4 | paper_rhs.m | 종·횡·yaw·roll, 초기상태 [u,v,r,phi,p] |
| Eq.5 | paper_rhs.m | ratio slip angle, 전륜 조향 반영 |
| Eq.6–10 | paper_rhs.m | Burckhardt, lambda_long=0 가정 |
| Eq.11 | paper_rhs.m | ay/Fz 결합의 접촉별 해석해, 공개 수식·새 수치 해법 |
| Sec.3.1 | paper_simulate_grid.m | yaw OR ay OR wheel lift; 5초·dt는 가정 |
| Fig.4–5 | paper_build_region.m | roll/lateral 한계점의 실제 안정점 8개 선택; 상세 순위는 가정 |
| Fig.6 | run_requested_40kph.m | 요청 0/3/7°, 논문은 0/3.5/7° |

기존 PCA의 격자 밖 외삽과 미수렴 하중 반복을 수정했습니다. 부피는 m/s·rad/s·rad 단위의 상태공간 부피이며, 물리적인 m³가 아닙니다. 논문 미공개 파라미터·기하 순위·시간 및 속도 운용 가정은 README에 공개했습니다.
