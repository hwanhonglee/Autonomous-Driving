# 주행·지도 모델 학습 완료 결과

<!-- HH_260906 - Auto-publish verified completed summaries without deployment or feature approval. -->

두 학습과 평가가 끝났습니다. 자동 채택·배포는 하지 않았습니다.

## 주행 모델

- 우회전 평균 궤적 오차: 3.7037 m.
- 직진 평균 궤적 오차: 0.6008 m.
- 기존 K1 대비 비퇴행: 6/9. Strict gate: 6/9.
- 기존에 반복 관측한 DEV 평가이며 실차·폐루프 승인 아님.

## RGB 지도 인식 모델

- 평가 mIoU: 0.5063; 픽셀 정확도: 0.9346.
- 같은 Town/날씨의 별도 경로 파일럿. 독립 TEST 또는 완성된 정밀지도 품질 아님.
- [실제 학습 모델 예측 GIF](mapping/learned_perception.gif)

| 분류 | IoU |
|---|---:|
| lane_marking | 0.8454 |
| other | 0.8807 |
| pole | 0.0688 |
| road | 0.9582 |
| sidewalk | 0.7906 |
| traffic_light_fixture | 0.0002 |
| traffic_sign | 0.0000 |
