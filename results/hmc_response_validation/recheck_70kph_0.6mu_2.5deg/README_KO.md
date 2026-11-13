# 교체된 70 km/h CarSim 파일 재비교

새로 올린 `CarSimOut_trim_70_0.6_2,5.mat`을 동일한 수정 독립 모델의 10초 응답과 다시 비교했습니다. 조건은 70 km/h, μ=0.6, 전륜 조향 명령 2.5°, 정상 타이어, 정상 스프링, CDC 950 mA이며 초기 횡속도 0.1 m/s·차체 요레이트 0.35 rad/s입니다. CarSim 실제 전륜 평균각은 약 2.222165°입니다.

새 자료의 첫 행은 횡속도 0.099359274 m/s·요레이트 0.35 rad/s입니다. 이전 파일에 있던 첫 trim 행이 없어졌고, 전체 72채널에서 새 파일의 마지막 행 제외 배열과 이전 파일의 첫 행 제외 배열이 정확히 일치합니다. 즉 같은 물리 응답이 1 ms 앞당겨 기록됐습니다. 운전 조건과 물리 초기조건이 같으므로 기존 모델 궤적을 재사용했고, 모델 곡선을 이동하거나 기록 시간을 맞춰 보정하지 않았습니다. 새 파일의 0~10초 10,001개 행을 모두 비교했습니다.

| 지표 | CarSim | 수정 독립 모델 |
| --- | ---: | ---: |
| 롤 최대값 (°) | 2.467510 | 2.124729 |
| 10초 롤각 (°) | 2.017953 | 1.811959 |
| 10초 피치각 (°) | 0.040915 | 0.043770 |
| 10초 요레이트 (rad/s) | 0.198620 | 0.204910 |

롤 RMSE **0.211302°**, 피치 RMSE **0.005053°**, 요레이트 RMSE **0.006483 rad/s**입니다. 기록 시작이 바뀌어도 롤 최대값과 정상상태 차이는 남아 있습니다. 적분 간격 검사와 모델 구현 정보는 [이전 비교 설명](../README_KO.md) 및 [수치 검증](../numerical_verification.json)에 있습니다.

- [비교 MAT](comparison_70kph_0.6mu_2.5deg.mat)
- [비교 그래프](comparison_70kph_0.6mu_2.5deg.png)
- [전체 오차 지표](comparison_70kph_0.6mu_2.5deg_report.json)

MAT의 `ResponseComparison`에는 선택 응답 6개·4코너 스프링/댐퍼 힘·조건·오차 지표·해시만 저장했습니다. 원본 차량·타이어 계수나 전체 72채널 기록은 포함하지 않았습니다. 210조건 계산은 중단 상태를 유지합니다.

```matlab
S = load('comparison_70kph_0.6mu_2.5deg.mat');
R = S.ResponseComparison;
plot(R.time_s,R.carsim_response.roll_deg, ...
     R.time_s,R.independent_response.roll_deg,'--');
legend('CarSim','Independent model'); grid on;
xlabel('Time [s]'); ylabel('Roll angle [deg]');
```
