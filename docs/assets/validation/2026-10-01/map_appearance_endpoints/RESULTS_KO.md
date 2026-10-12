# 조명 조건을 맞춘 학습: 최종 비교 결과

<!-- HH_261001 - Complete fixed evaluation results; no model promotion or HD-map completion claim. -->

조명 적용 RGB 학습은 CTrack의 차선 인식을 개선했지만, 직진 도로 분류까지
안정적으로 복구하지는 못했습니다. 구조·초기 가중치·학습량·프레임·라벨·크롭은
동일하며, Control은 새480장의 후처리=false, Appearance는 같은 프레임의=true를
학습했습니다. 각 모델은 정확히10,000스텝입니다.

IoU(%), **Control → Appearance**:

| CTrack 경로 | 도로 | 차선 표시 | 보도 |
| --- | ---: | ---: | ---: |
| LEFT | 44.26 → 78.04 | 3.15 → 21.09 | 0.78 → 29.10 |
| STRAIGHT | 31.06 → 27.68 | 4.45 → 26.93 | 2.59 → 5.81 |

STRAIGHT에서는 실제 도로 픽셀의66.30%를 보도로 오분류했습니다. 고정 영상에서도
중앙선 개선과 도로·보도 혼동이 함께 보입니다. 기존 조명 성능도 유지하지 못했습니다:
Town01 DEV와 Town02 p02 holdout 모두 **7개 클래스 IoU가 Control보다 하락**했습니다.
새 합격 기준을 사후 적용한 것이 아니라, 관측된 성능 하락입니다.

반면 조명 적용 TRAIN 원본에서는 Appearance 도로 IoU가 Town01 99.51%, Town02
99.57%이고 차선은83.25%/83.96%입니다. 이는 학습 프레임 재평가 성적일 뿐,
미관측 지도 일반화의 증거는 아닙니다. 더 큰 헤드나 무작정 추가 학습보다는
색·명암 의존성과 재질 차이에 대한 통제 실험이 타당합니다.

검증은 네 시나리오 전체240프레임씩을 사용했습니다. **2,880개 예측·663,552,000픽셀**을
독립 재집계했고, 고정0/119/239의 비교 PNG12장에 들어간 **원본 크기 패널72개**도
픽셀 단위로 확인했습니다. 기존 DEV/holdout의161,280,000픽셀은 별도로 재집계됐습니다.
CPU2만 사용했고 평가 중 추가 학습·GPU·CARLA 실행·프레임 선별은 없었습니다.

- [전체 수치·12개 고정 PNG](RESULTS.md)
- [LEFT 고정119 비교](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_appearance_endpoints_ctrack_left_20261001_v1/ctrack_left_119_full_fov.png)
- [STRAIGHT 고정119 비교](/home/a/autoware_e2e/storage/experiments/validation/2026-10-01/map_appearance_endpoints_ctrack_straight_20261001_v1/ctrack_straight_119_full_fov.png)
- [다음 통제 실험 제안](NEXT_PHOTOMETRIC_PROPOSAL.md)

CTrack은 계속 EVAL 전용입니다. 이전에 관찰한 실패가 실험의 동기였으므로 독립 인증은
아닙니다. LEFT의 원래 owner FAIL과 live-map 동일성 확인 불가는 그대로 보존했습니다.
**모델 자동 승격, HD-map 완성 또는 전체 자율주행 완료를 의미하지 않습니다.**
