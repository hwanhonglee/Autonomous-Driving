# CTrack 전체 기록 누적 결과 — 도로는 개선, 차선·오검출은 미해결

<!-- HH_261001 - Report matched full-recording evidence, inspected visuals and remaining errors without changing frozen artifacts. -->

**기존 이미지 분할 개선이 실제 누적 지도에서도 도로 셀 개선으로 이어졌다. 그러나 차선 누락과 도로 주변 오검출은 남았고, 일부 오검출은 증가했다.** CTrack LEFT·STRAIGHT 각각240장 전량을 두 모델로 처리했다. 새 학습이나 촬영은 없다.

현재 상태: **전체 누적 완료·별도 독립 재생 PASS**. [원본 summary](run_v1/summary.json)와 [독립 재생 영수증](run_v1/independent_replay.json)을 확인했다. 독립 재생은188.84초, 새 모델 forward0회로 모든 중간/최종 map 상태와 PNG를 재현했다. 원본 summary의 당시 `PENDING` 표시는 그대로 두고 별도 영수증으로 완료를 기록한다. 모든 frozen source·PLAN·산출물은 수정하지 않는다.

## 무엇을 비교했나

현재 geometry 연구의 **같은 조건으로 새로 학습한 control → geometry** 비교다. 두 팔의 차이는 추가480 slot에 기존 Town01 FALSE 경로를 재노출했는지, 새로 승인된 p02/p04 FALSE 원본을 넣었는지다. 예전 photometric 모델이나 0·119·239의 **세 프레임만 누적한 결과와 같은 비교가 아니다**.

전체480장, CPU2 **960회 시도·960회 완료**, learned map update960회, oracle update480회, 총334.27초였다. 모든 이미지의 argmax가 기존 저장 마스크와 정확히 일치했고, 전체 confidence와 저장한 stride3 float32 confidence의 투영점도 일치했다. 이는 기록 재처리이며 실시간10Hz 측정이 아니다.

RGB는 모델이 분할하지만 위치 계산에는 **CARLA 정답 depth·pose**를 사용한다. 기존 confidence≥0.5, stride3, 깊이(.1,40)m, 0.2m 셀, 최대200,000셀, 프레임별 최대 confidence 누적법을 바꾸지 않았다. 높이 필터·정답 기반 learned-pixel 제외·빈 영역 채우기도 없다.

## 최종 BEV 셀 결과

아래 IoU는 **이미지 픽셀이 아니라, 전체 기록을 누적한 셀의 대표 클래스** 기준이다. 각 수치는 control → geometry, 단위는 %다.

| 경로 | road IoU | lane_marking IoU | sidewalk IoU |
| --- | ---: | ---: | ---: |
| LEFT | 72.10 → **77.25** | 8.62 → **9.34** | 37.22 → **39.70** |
| STRAIGHT | 68.45 → **71.54** | 20.08 → **21.88** | 24.21 → **24.95** |

도로 정답 셀 중 정확히 road로 누적된 수는 LEFT **23,391→25,569 /29,785**, STRAIGHT **20,517→21,775 /27,620**이다. 반면 정확한 차선 셀은 LEFT **122→134 /1,375**, STRAIGHT **247→273 /1,172**뿐이다. road로 흡수되거나 관측되지 않은 차선이 여전히 많아, 차선 연결·경계 복원을 완료했다고 볼 수 없다.

| 범위·오검출 지표 | LEFT | STRAIGHT |
| --- | ---: | ---: |
| 전체 oracle 정적 셀 수 | 39,487 | 35,289 |
| learned 셀 수 | 41,320 → 44,560 | 35,384 → 36,766 |
| oracle 셀 중 learned 관측 비율¹ | 86.85% → 90.99% | 88.78% → 91.62% |
| 관측 셀 교집합/합집합 | 73.73% → 74.67% | 79.63% → 81.40% |
| oracle 정적 셀 밖 learned 셀² | **7,025 → 8,630** | **4,055 → 4,433** |
| road 오검출 셀 | **2,656 → 3,316** | **2,354 → 2,818** |
| sidewalk 오검출 셀 | 4,466 → 4,060 | 6,504 → 6,145 |
| pole 오검출 셀 | **5,446 → 6,206** | **2,165 → 2,187** |

¹ 클래스가 맞는지와 별개인 관측 범위다. ² oracle의 여섯 정적 클래스에 없는 셀이며, 그 장소가 물리적으로 빈 공간이라는 뜻은 아니다. **셀 수 증가를 곧바로 정확한 지도 면적 증가로 해석하면 안 된다.**

lane 오검출도 LEFT40→60, STRAIGHT58→76으로 증가했다. oracle에서 pole·traffic-light fixture·sign의 지지 셀이0이므로 해당 IoU는 **null**을 유지한다. fixture 오검출은 LEFT370→155 /STRAIGHT128→61로 줄었지만 sign은68→142 /264→277로 늘었다. 없는 클래스를0점이나 완벽한 정답으로 대체하지 않았다.

## PNG 8개 직접 확인

2026-10-01 06:38 KST에 아래 **8개 PNG 전체, 각3개 패널**을 원본 해상도로 확인했다. 왼쪽은 oracle semantic, 가운데 control, 오른쪽 geometry다. 청회색은 road, 보라색은 sidewalk, 노란색은 lane marking, 회백색은 pole이다. 전체 경로 그림의 **흰 선은 정답 차량 이동 경로이지 예측 차선이 아니다**.

| 경로 | 시작 | 중간 | 마지막 | 모든 셀·전체 경로 |
| --- | --- | --- | --- | --- |
| LEFT | [000](run_v1/ctrack_left_000_bev.png) | [119](run_v1/ctrack_left_119_bev.png) | [239](run_v1/ctrack_left_239_bev.png) | [전체](run_v1/ctrack_left_full_route_bev.png) |
| STRAIGHT | [000](run_v1/ctrack_straight_000_bev.png) | [119](run_v1/ctrack_straight_119_bev.png) | [239](run_v1/ctrack_straight_239_bev.png) | [전체](run_v1/ctrack_straight_full_route_bev.png) |

LEFT는 교차부와 굽은 도로의 큰 형태가 남지만, oracle의 여러 차선·가장자리 표시가 learned 지도에서 끊기거나 사라진다. STRAIGHT도 중심선 일부는 이어지지만 가장자리 차선이 부족하고 road 일부가 sidewalk로 번진다. 두 경로 모두 도로 주변에 수목 가지 모양의 pole 오검출과 작은 잘못된 클래스 조각이 누적된다. geometry가 이 문제를 제거한 것은 아니다.

000 그림의 먼 거리 가로줄은 희소 투영 격자 흔적이지 정지선 복원 증거가 아니다. 고정 ego 그림은 오래된/화면 밖 셀을 잘라 보이므로, 전체 경로 그림도 함께 확인했다. 전체 그림은 세 모델의 모든 저장 셀과240개 pose를 포함하는 공통 범위·배율이며, 어두운 부분은 **미관측/unknown**이다. 정밀 값은 각 경로의 oracle/control/geometry `*_grid.npz`에 전량 남아 있다.

## 검증·해석 경계

- 별도 verifier가 저장 confidence·마스크·depth·pose만으로 **새 forward 없이** 1,440개 증분 map state, 최종6개 grid, PNG8개의 픽셀과 해시를 재검증하여 **PASS**했다. 프레임별 cell/class 최대 confidence fusion도 독립 구현으로 대조했다. exact-argmax는 원 실행에서 확인한 assertion이며 no-forward verifier가 모델 출력을 다시 계산하는 것은 아니다.
- LEFT는 **원래 owner FAIL·당시 live map replay 불가**를 그대로 유지한다. 이번 forensic raw 기반 누적과 독립 재생이 성공해도 과거 실패를 PASS로 바꾸지 않는다.
- [이미지 결과](../map_geometry_endpoints/RESULTS.md)의 STRAIGHT road62.47→72.33%와 여기의 BEV road68.45→71.54%는 분모·confidence 선택·시점별 가중·fusion이 달라 직접 같은 점수로 비교하지 않는다. known350의161,280,000픽셀 재집계도 별도 증거다.
- 높이 필터가 없어 수직 mesh와 의미 오분류도 x/y에 투영된다. CARLA mesh tag 자체도 완벽한 지면 정답이 아니다. 관측 범위 밖 도로·차선, 차선 topology, 독립 검증, 주행 기능, 실제 센서, camera-only mapping 또는 **HD-map 완성**을 입증하지 않는다. CTrack은 계속 EVAL 전용이며 자동 승격은 없다.

## 차선 누락은 어디에서 생겼나

<!-- HH_261001 - Separate accepted-observation gaps from final fusion losses without changing the map recipe. -->

[최종 격자6개를 별도로 집계한 진단](lane_evidence_diagnosis_v1/DIAGNOSIS.md)도 완료했다. 새 모델의 LEFT 정답 차선1,375칸은 **관측 없음116 / 관측됐지만 채택된 차선 증거 없음1,048 / 차선 증거가 다른 라벨에 밀림77 / 차선으로 맞음134**로 나뉜다. STRAIGHT1,172칸은 **79 / 801 / 19 / 273**이다. 네 범주 합계는 각 분모와 정확히 일치하며13개 테스트도 통과했다.

즉, 마지막 누적 투표에서 차선이 밀리는 경우보다 **그 칸에 채택된 차선 증거 자체가 없는 경우**가 훨씬 많다. 다만 confidence·희소 샘플링·깊이 범위·격자화도 개입하므로, 이것만으로 원본 영상의 분류 실패라고 단정할 수는 없다. 차선을 한 번이라도 관측하면 무조건 우선하는 변경도 정답 기준 오검출을 크게 늘릴 수 있어 적용하지 않았다. 일부 기준 오검출에는 oracle의 보조 차선 증거가 있어 물리적인 허구 차선과도 구분한다.

## 다음 학습 우선순위 한 가지

**공간·수집 승인을 확보한 뒤, 이미 한정 TRAIN으로 승인된 Town01 p02/p04의 고정 경로를 동일 rig로 FALSE/TRUE 동시 촬영하는 데이터 보강**이 우선이다. 기존 새 경로480장은 FALSE 외관뿐이다. 같은 프레임·같은 semantic/depth/pose에 두 RGB를 묶고 전량 보존하면, 새 도로/연석 기하와 실제 TRUE 조명·그림자·얇은 차선·수목 배경을 함께 제공할 수 있다. CTrack 오답 장면을 TRAIN으로 가져오거나 점수로 프레임을 고르지 않는다.

현재 paired TRUE TRAIN road는 약99.2–99.3%인데 외부 도로·차선·수목 오검출은 크게 남는다. 이는 데이터 범위/외관 차이를 먼저 통제할 근거이지, 모델 용량이 충분하다는 최종 증명은 아니다. 단순 epoch 증가·head 확대·투영 임계값 조절보다 이 **같은 경로의 렌더 외관 쌍**을 먼저 확보하고 QA한 뒤, 같은 모델·노출량의 별도 matched fit을 검토한다. 이 문서는 추가 촬영·학습을 실행하거나 승인하지 않는다.

## 고정 증거

[사전 계획](PLAN.md) · [등록](preregistration.json) SHA `ec54254aa275e9bee03b31cc1ba3325baa6ff2921a422f210a71c473b96c7721`.

[원본 summary](run_v1/summary.json) SHA `b848700f12f740d1afb424187e4e5ef8fe3eb8af6819629b625da03da9397fd1`. 독립 재생 영수증 작성 전 산출물은77,994,817bytes로128MiB 상한 이내다. 원본 RGB 복사·삭제 없이5GiB free-disk floor를 유지한 guarded write를 사용했다.

[독립 재생 PASS](run_v1/independent_replay.json) SHA `bb8149546f098cbdfaab5a35187a237bf51c6be81df4d04007bcf2900fc0a574`. source/input 해시 불변,960개 learned·480개 oracle 투영,1,440개 증분 상태,6개 최종 grid의 label 순서/동률,8개 PNG를 모두 확인했다.
