# Town02 자료 추가: 도로·차선 지도 모델 A/B 결과

<!-- HH_260929 - Report both the useful road signal and failed lane/generalization gates without promoting a model. -->

**결론: D1 미채택, 기존 AI v2 유지.** 개인 PRO6000 물리 GPU 0에서 D0와 D1을 각각 2,000 step 학습했다. D0는 보관 AI v2의 Town01 DEV 7클래스 IoU를 모두 **정확히 재현**했다. D1은 같은 조건에서 Town02_Opt TRAIN 232장만 추가한 실험이다. Town05_Opt DEV의 도로 픽셀 성능은 좋아졌지만 차선은 나빠졌고, Town01의 여러 클래스도 퇴행해 [사전 계약](PLAN.md)을 통과하지 못했다. 이 실험은 RGB 단일 프레임의 의미분할이며, 완성된 자동 정밀지도·신호등 연결 그래프·실차 운행 모델이 아니다.

| DEV 장소·IoU | D0: Town01 497장 | D1: +Town02 232장 | 변화 |
| --- | ---: | ---: | ---: |
| Town05_Opt 도로 | 36.31% | 52.30% | **+15.99%p** |
| Town05_Opt 차선 표시 | 9.01% | 7.33% | **−1.68%p** |
| Town01 도로 | 98.75% | 98.38% | −0.37%p |
| Town01 차선 표시 | 91.23% | 89.36% | −1.87%p |
| Town01 신호등 설비 | 15.18% | 10.79% | −4.39%p |

Town05 목표는 도로·차선 **각각 D0보다 +5%p**, 절대 도로 ≥80%·차선 ≥50%였다. 도로의 상대 개선 하나만 통과했고 절대 성능과 차선 목표는 실패했다. Town01 도로·차선의 사전 절대 하한은 넘었지만 둘 다 퇴행했고, 보도·신호등 설비도 퇴행했다. 따라서 `all_town01_classes_non_regressing=false`, `road_lane_development_target_met=false`, `model_promotion=false`다. Town05 표지판 IoU는 양쪽 모두 0이며, Town02의 이전 작은 표지판 원본 적격 실패(0/232)를 이번 실험으로 뒤집지 않는다.

## 화면 확인

[Town05 DEV 프레임 000](visuals_town05_dev_v1/town05_dev_000_road_lane_ab.png), [115](visuals_town05_dev_v1/town05_dev_115_road_lane_ab.png), [231](visuals_town05_dev_v1/town05_dev_231_road_lane_ab.png)의 PNG와 [3프레임 GIF](visuals_town05_dev_v1/town05_dev_road_lane_ab.gif)를 남겼다. 각 장은 **전체 640×360 시야**의 RGB·CARLA 정답·D0·D1, 그리고 차량 전방 근접 영역을 나란히 보여 준다. 표시는 도로 청색, 차선 황색이며 정답은 추론 입력이 아니다. 시각화 [manifest](visuals_town05_dev_v1/manifest.json) SHA256은 `95aec0fbd2fa29eaccee552ca8e54b11b13293093db40f5bf5d47bb58bbdffca`다. 화면에서도 D1 도로 영역이 더 이어지는 장면이 있지만 차선 누락과 건물·노면 오탐이 보인다. 정지 영상 비교로 실제 주행 결과를 주장하지 않는다.

[전 232장 영역별 오류 분석](ERROR_ANALYSIS.md)에서는 **231/232장의 전체 차선 IoU가 D1에서 감소**했다. 차량 전방 중앙의 정답 차선 적중 픽셀은 D0 1,648→D1 48로 줄었지만 D0 자체도 그 영역에서 매우 약했다. 먼 영역의 도로 오탐은 증가, 근접 영역의 도로 오탐은 감소해, 차선 손실을 하나의 단순한 치환 원인으로 설명할 수 없다. [프레임별 CSV](error_analysis_v1/per_frame.csv)와 [집계 JSON](error_analysis_v1/summary.json)은 DEV 진단용이며 독립 반복 실험이 아니다.

![Town05 DEV 전체 시야와 도로·차선 A/B 및 차량 전방 확대](visuals_town05_dev_v1/town05_dev_115_road_lane_ab.png)

## 재현과 출처

- [데이터 패키지 manifest](../../../../../storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2/manifest.json) SHA256 `313b41ed4fb897f01fb4fb433c1bafb234ef9b234e62f96a462ef2188e9dccc6`: D0 TRAIN 497, D1 TRAIN 729, Town01 DEV 118, Town05 DEV 232. 두 팔에 동일한 D0 클래스 가중치·seed·crop·2,000 step을 적용했다. Town05는 이미 분석에 사용된 **DEV**이며 새로운 블라인드 TEST가 아니다.
- [원격 실행 복사본](../../../../../storage/experiments/training/2026-09-29/map_road_lane_ab_run_v1/summary.json): `started.json`, 양쪽 checkpoint·예측 NPZ·혼동행렬·히스토리·소스 스냅샷 보존. D0 checkpoint SHA256 `39b81fe442cc045df36633c9d9603d06ac075babb4677d8ba911e6048b8fb413`; D1 `c583be21a082fbfaa0a9ed1346dc667f032ef928582bd8c537b949039404c38f`. 원본 AI v2는 교체하지 않았다.
- [독립 픽셀 재계산기](../../../../../tools/e2e/audit_map_road_lane_ab_result.py)는 보관 예측과 DEV 라벨의 모든 픽셀로 네 혼동행렬·7클래스 IoU를 다시 계산해 **PASS**했다. 데이터·소스·체크포인트 해시도 일치했다. 시각화 생성기도 입력·혼동행렬·미채택 상태를 별도 검사했다. 집중 테스트는 지도 패키지/훈련기/감사와 주행 경로 감사 **20개**, 시각화 **6개**, 오류 분석 **4개**로 합계 **30개 PASS**였다.
- 두 팔 GPU 훈련 시간은 각각 76.46초, 75.87초(합계 152.33초)였다. 전송·사전 검사·평가·시각화 시간은 별개다. 실행 종료 때 두 물리 GPU 메모리 사용량은 모두 0으로 돌아왔다. GPU 1에는 학습을 배치하지 않았고 개인 venv 밖에 설치하지 않았다.

## 해석과 다음 단계

이번 한 seed 결과는 Town02 자료의 *도로 배경 일반화 신호*와 동시에 *차선 보호 실패*를 보여 준다. D1에 기능 head를 더 붙이거나 이 체크포인트를 배포하지 않는다. 다음 지도 실험은 퇴행이 집중된 거리·영역을 기준으로 차선 라벨/경계 정렬 및 Town 간 카메라·노면 외관 차이를 **TRAIN만으로** 점검하고, 별도의 미사용 장소와 최소 3 seed가 마련된 뒤에만 단일 요인 A/B를 사전 등록한다. 같은 Town05 DEV를 반복해서 최적화하지 않는다. Town13 RGB는 skydome 경고와 source-editor 스트림 충돌 때문에 여전히 학습/DEV에서 제외한다. 지도 모델 결과를 주행 모델에 공급하는 A/B도 아직 하지 않았다.

재검사 명령은 로컬 작업공간 루트에서 다음과 같다.

```bash
python3 tools/workspace_python.py tools/e2e/audit_map_road_lane_ab_result.py \
  --data storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2 \
  --run storage/experiments/training/2026-09-29/map_road_lane_ab_run_v1
```
