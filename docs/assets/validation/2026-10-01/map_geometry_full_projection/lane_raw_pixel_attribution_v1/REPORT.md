# 차선 누락의 한 단계 앞: 원본 픽셀에서 무엇을 예측했나

<!-- HH_261001 - Attribute saved lane misses at matching sampled oracle pixels; preserve all frozen results. -->

**결론:** CTrack 전체240프레임씩에서 차선 정답 셀에 채택된 차선 증거가 없는 주된 이유는, 같은 픽셀의 모델 `argmax`가 차선이 아니라 다른 클래스, 특히 **도로**였기 때문이다. 기존 `confidence≥0.5` 필터나 최종 셀 투표가 이 범주의 주원인은 아니다. 이는 저장된 CARLA semantic 기준의 사후 진단이지, 실제 도로에서의 원인을 증명하거나 HD map을 승인하는 결과가 아니다.

## 비교 기준과 재현

[읽기 전용 재현 코드](../../../../../../tools/e2e/diagnose_map_lane_raw_pixels_20261001.py)는 새 모델 추론·학습·CARLA 실행·임계값 변경·파일 출력을 하지 않는다. 실행 명령:

```bash
python3 -B tools/e2e/diagnose_map_lane_raw_pixels_20261001.py
```

기존 [full240 summary](../run_v1/summary.json) SHA256 `b848700f12f740d1afb424187e4e5ef8fe3eb8af6819629b625da03da9397fd1`, [사전 등록](../preregistration.json) SHA256 `ec54254aa275e9bee03b31cc1ba3325baa6ff2921a422f210a71c473b96c7721`을 기준으로 했다. 재현 코드 SHA256은 `c92d142737f439f12bddf282a18677c2dacea6b407198d4e5d49030911e66c91`이다. 원본480개 NPZ, 저장 예측4개, stride3 최대 softmax confidence960개, 최종 격자6개의 SHA를 각 frozen receipt와 대조했다. 투영 소스 `mapping_projection.py` SHA `c42ab7d20454475f544b8073d6a23a104a0a2bb1cc2cbb7be092fa3e871cbbee`, 융합 소스 `mapping.py` SHA `264f9dda759f6986bde9c614e5cf3ae21680a22ae58d855db11564ceb3b3b867`도 확인했다.

분모는 **최종 oracle 격자에서 차선이 우세한0.2m 셀**로, LEFT1,375칸·STRAIGHT1,172칸이다. 이 셀을 만든 같은 stride3 격자의 CARLA semantic 차선 픽셀과 저장된 모델 class/confidence를 동일 프레임·좌표에서 맞췄다. 이전 [최종 격자 진단](../lane_evidence_diagnosis_v1/DIAGNOSIS.md)의 네 그룹(a 미관측, b 관측됐으나 차선 증거 없음, c 차선 증거가 투표에서 패함, d 차선 우세)을 유지했다.

| 경로 / 모델 | a | b | c | d | b 셀의 동일 정답 픽셀에서 차선 `argmax` |
| --- | ---: | ---: | ---: | ---: | ---: |
| LEFT control | 175 | 989 | 89 | 122 | **0/989셀** |
| LEFT geometry | 116 | 1,048 | 77 | 134 | **0/1,048셀** |
| STRAIGHT control | 116 | 785 | 24 | 247 | **0/785셀** |
| STRAIGHT geometry | 79 | 801 | 19 | 273 | **0/801셀** |

따라서 b셀에서는 낮은 *차선 승자 확신도* 때문에 버린 사례가 **0셀**이다. STRAIGHT geometry의 a셀79칸 중5칸에는 차선 `argmax`가 있었지만 저장된 최대 확신도가0.5 미만이었다. 그 외 a셀과 모든 b셀은 같은 정답 픽셀에서 차선 `argmax` 자체가 없다. c/d셀에서는 각각 동일 정답 픽셀의 채택된 차선 예측이 실제 최종 차선 증거와 일치했다.

## b셀에서 모델은 무엇을 보았나

다음 분모는 위 **b셀에 속한 stride3 CARLA 차선 픽셀 수**로, 셀 수와 다르다. 한 셀에 여러 프레임·픽셀이 들어갈 수 있다. `confidence`는 저장된 **승자 클래스의 최대 softmax**이다. 차선이 패배한 픽셀의 차선 클래스 확률은 저장되지 않았다.

| 경로 / 모델 | b 픽셀 | road로 예측 | road 비율 | 다른 예측(원본 픽셀 수) | 승자 confidence q10 / 중앙값 / q90 |
| --- | ---: | ---: | ---: | --- | --- |
| LEFT control | 46,057 | 44,231 | 96.04% | sidewalk1,351 · other434 · pole30 · sign11 | .9959 / 1.0000 / 1.0000 |
| LEFT geometry | 46,234 | 45,351 | 98.09% | sidewalk583 · other288 · pole7 · sign5 | .9987 / 1.0000 / 1.0000 |
| STRAIGHT control | 30,946 | 23,894 | 77.21% | sidewalk3,431 · other2,941 · pole672 · sign8 | .6956 / .9993 / 1.0000 |
| STRAIGHT geometry | 31,532 | 25,410 | 80.58% | sidewalk2,804 · other2,916 · pole394 · sign8 | .7369 / .9996 / 1.0000 |

geometry는 전체 차선 TP를 LEFT122→134, STRAIGHT247→273셀로 조금 늘렸지만, 여전히 b셀의 정답 차선 픽셀은 주로 road로 분류했다. 특히 **선택된 모델의 b셀 road 비율은98.09% /80.58%**다. 이는 분류 출력의 문제를 구체화할 뿐, 왜 그런 분류가 나오는지(학습 데이터, 외관 차이, semantic mesh 오류, 모델 구조 중 무엇인지)까지 단정하지 않는다.

## stride·깊이·투영 경계

원본 CARLA 차선 태그에서 깊이(.1,40)m가 유효한 픽셀은 LEFT993,519개, STRAIGHT841,148개였다. 원래 stride3에서 실제 샘플된 것은 LEFT109,920개(11.064%), STRAIGHT93,204개(11.081%)다. **샘플 밖 차선은 원래 oracle 셀 분모에도 들어가지 않는다.** 따라서 현재 b셀의 차선 누락을 `stride3가 해당 픽셀을 건너뛰어서`라고 설명할 수 없다. 반면 원본 전체 차선 면적의 포괄적 지도화는 이 별도 샘플링 한계를 갖는다.

같은 깊이·pose를 사용한 oracle float64와 learned float32 깊이 변환의0.2m 셀 차이는 샘플된 정답 차선 픽셀에서 **LEFT0/109,920, STRAIGHT0/93,204**였다. 같은 좌표의 채택된 차선 예측이 b셀에서 사라진 투영 차이는 관측되지 않았다. 이것은 CARLA 정답 depth·pose가 실제 센서에서도 정확하다는 뜻이 아니며, 수직 물체의2D 격자 혼합이나 이웃 픽셀·이웃 셀 충돌 가능성까지 없애지 않는다.

## 다음 실험을 위한 제한된 해석

현재 증거는 단순 차선 우선 투표, confidence 문턱 완화, stride 변경만으로 **이 b셀을 바로 회복할 수 있다는 주장을 지지하지 않는다**. 학습용으로 승인된 새 p02/p04의 동일 경로 TRUE/FALSE 원본 쌍과 얇은 차선의 수동 영상 확인이 다음 데이터/라벨 진단의 합리적인 후보이지만, 본 분석은 어떤 새 데이터도 TRAIN에 넣거나 새 모델을 승격하지 않는다. LEFT의 원래 owner FAIL과 live replay 불가도 그대로다.
