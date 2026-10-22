# MATLAB 안정영역 계산 코드

파일 하나로 실행하려면 [stability_sweep_210.m](stability_sweep_210.m)만 저장한 뒤 MATLAB에서 `stability_sweep_210`을 실행하세요. 모델·초기화·적분·영역 추출·저장 함수가 모두 들어 있습니다. 결과는 파일 옆 `results_single_mfile_gridbox` 폴더에 저장합니다. 다른 `.m` 파일이나 C++ 실행 파일은 필요하지 않으며, 자기 파일 해시 계산에는 MATLAB Java VM을 사용합니다.

현재 v4 코드는 초기 횡속도·요레이트·바퀴 조향각을 기존 `calculate_internal_slips` 함수에 넣어 초기 슬립각을 구합니다. 초기 휠 회전속도는 `max(u_wheel0,0)/Rw`, 초기 종방향 완화 슬립은 0입니다. 시뮬레이션 중 사용하는 바퀴 운동학과 초기화가 같은 계산식을 사용합니다. 이전 `-delta_w` 초기화 결과와는 초기조건 정의가 다르므로 이전 MAT를 새 코드의 계산 캐시로 혼용하지 마세요.

기본 실행은 속도 50·60·70·80·90·100 km/h, μ 0.9·0.8·0.7·0.6·0.5, 앞 휠 조향각 0·0.5·1·1.5·2·2.5·3°의 210조건과 마모 5종입니다. 원본 초기조건 39,401개(41×31×31), 관측 5초, Ts=0.01초, RK4 하위 적분 20회를 유지합니다. 파일명은 `stable_point_80kph_0.9mu_2deg.mat` 형식입니다.

요청한 정상 타이어 세 조건만 계산하려면 다음 두 호출을 사용하세요. 정상 타이어 결과의 `STABILITY3D.CASE`는 하나이며 `meta.healthy_only=true`입니다.

```matlab
stability_sweep_210(struct('speeds',80,'mus',0.9,'steers',2,'wear_cases',[0 0 0 0]));
stability_sweep_210(struct('speeds',50,'mus',0.9,'steers',[0 2],'wear_cases',[0 0 0 0]));
```

ZIP의 여러 파일을 사용하는 방식은 `run_210_conditions.m`을 실행하면 됩니다. 이 진입점은 순수 MATLAB M-code를 사용하며 결과를 `results_mcode`에 저장합니다. 클라우드는 같은 모델의 C++ 가속 경로를 사용합니다. 가속기 소스 `native_accel.cpp`와 빌드 스크립트도 포함했습니다. 새 초기화에 대해 MATLAB·C++ 초기 상태, RHS, RK4, 짧은 궤적을 대조한 47개 검사를 통과했습니다.

노면 μ는 상대 배율 `mu/0.9`로 종·횡 타이어 Magic Formula 진폭 `Dx`, `Dy`와 결합 슬립 계산에 반영합니다. μ=0.9에서 원본 타이어 보정을 유지합니다. 요율 한계는 `0.85*mu*g/ux`, 횡가속도 한계는 `mu*g`, LTR 한계는 1입니다. 횡력 오프셋 `SVy`는 그대로 두므로 실제 피크를 모든 하중·마모에서 정확히 `mu*Fz`로 강제하는 절대 정규화는 아닙니다.

기존 영역 추출이 실패할 때만 모든 내부 격자점이 안정인 최대 축 정렬 박스를 후순위로 사용합니다. `inner_method=largest_all_stable_grid_box`와 후처리 버전으로 구분합니다. MAT를 다시 시뮬레이션하지 않고 보려면 `plot_stability_mat(파일명)`을 사용하세요. 이 그림 함수는 warning에 마모 벡터 대신 스칼라 네 개를 전달해 서식 오류를 방지합니다.
