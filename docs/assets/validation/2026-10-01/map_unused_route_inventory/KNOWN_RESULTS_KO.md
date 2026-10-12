# 새 경로480 학습: 알려진 평가 경로 결과

<!-- HH_261001 - Report all matched known-split gains and regressions without model promotion or independent-validation claims. -->

**원본·학습 영수증·전체 픽셀 독립 검증은 PASS, 성능은 혼재합니다.** Town01 DEV는 7클래스 IoU 모두 좋아졌지만, Town02 p02에서는 도로·보도·pole·other가 하락했습니다. 새 geometry 모델을 자동 채택하지 않습니다.

두 팔은 원래 context-B에서 각각 새로10,000 step/80,000표본을 학습했습니다. 공통729 FALSE+480 TRUE는 같고, 추가480 slot만 **control=기존 Town01 replay**, **geometry=미사용 p02/p04 전체480장**으로 바꿨습니다. 실제 학습 시간은424.38초/423.18초입니다. 초기 가중치·class weight·crop·샘플 slot·TRUE 전용 광도 정책은 동일합니다. 학습/평가 데이터나 모델을 이번 검증에서 추가 생성하지 않았습니다.

## 모든 클래스 IoU — control → geometry

단위는 %, 차이는 %p입니다. 기존에 반복 노출된 Town01 DEV118장·Town02p02 holdout232장이며 독립 DEV/TEST 인증이 아닙니다.

| 클래스 | Town01 control→geometry | Δ%p | Town02p02 control→geometry | Δ%p |
| --- | ---: | ---: | ---: | ---: |
| 도로 | 99.540 → 99.730 | +0.190 | 98.799 → 98.591 | −0.208 |
| 차선 | 96.884 → 97.482 | +0.599 | 97.541 → 98.535 | +0.994 |
| 보도 | 94.948 → 96.870 | +1.922 | 94.219 → 93.170 | −1.048 |
| 신호등 fixture | 44.511 → 66.298 | +21.787 | 31.109 → 36.395 | +5.286 |
| 표지판 | 7.926 → 8.212 | +0.287 | 70.867 → 74.314 | +3.447 |
| pole | 39.537 → 46.437 | +6.900 | 41.836 → 39.990 | −1.846 |
| other | 97.171 → 98.044 | +0.873 | 95.425 → 94.811 | −0.614 |

7클래스 단순 평균 IoU는 Town01 **68.645→73.296%**, Town02 **75.685→76.544%**입니다. 하지만 Town02 전체 픽셀 정확도는 **97.651→97.341%(−0.310%p)**로 낮아졌습니다. 평균 상승으로 도로/보도 퇴행을 가리지 않습니다.

Town01 표지판 IoU가 조금 올라가도 FP는 **15,191→19,417픽셀**로 늘었습니다. Town02의 FP도 도로 **45,186→74,530**, 보도 **393,571→496,725**, pole **352,756→431,537**로 증가했습니다. 작은 클래스의 연속 프레임 픽셀을 독립 객체 수로 해석하지 않습니다. 모든 class support·FP·정밀도는 [기계 판독 결과](KNOWN_RESULTS.json)에 있습니다.

## 검증 범위와 제한

- **350개 고정 프레임 × 두 모델 =700개 예측 /161,280,000픽셀**을 새 forward 없이 저장 mask와 기존 oracle target에서 전부 재집계했습니다. Native640×360 전체 화각이며 프레임/영역 제외가 없습니다. 두 split 모두 7클래스 GT가 존재했고, 검증기는 absent-class IoU null 규칙도 유지합니다.
- 36개 artifact 해시, 9개 학습 source, 두 팔의 각100개 loss 기록/10k step, 각각52개 float32 tensor/245,831개 parameter와 전체 state 해시, 동일 초기 상태·sample/crop/photo trace를 독립 확인했습니다. 모든 original729/TRUE480과 geometry 전체480의 노출도 일치합니다.
- `.pt`는 바이트 해시만 확인하고 pickle을 열지 않았습니다. GPU·추론·재학습·새400MiB 데이터 import는0입니다. 원격 전용 새 학습 배열의 자격은 고정된 package/preflight와 실행 당시 hash-check 영수증에 근거하며, 로컬에서 다시 열었다고 주장하지 않습니다.
- 이 PASS는 **재현 증거/정합 검증**입니다. 성능 승격 관문이나 새로운 허용 퇴행 기준을 만든 것이 아닙니다. CTrack 2종과 기존 TRAIN 2종의 별도 네 시나리오 평가 분모와 합치지 않으며, CTrack은 계속 EVAL 전용입니다. 여기서는 외부 성능·HD-map 완성·주행 성공을 결론내리지 않습니다.

## 정확한 해시

- 최종 summary: `caa1a60bcff8179b80de0a5861aa2ab0497d838638fbf80a3f4790e8074e11ef`
- [독립 재집계 영수증](known_split_independent_recount.json): `760861c1f4327f98c8a2c3a66b0a22f666a85100ff47bd2e026de69429a66558`
- Control NPZ: `7bd474649e0698497a5eef94430993626dc0e77c29bca4de759a5a9b46248416`
- Geometry NPZ: `70b5077c0e1c75f44b731692646e16ebcd784d8a3dc9a6c989d4934ffff1cd83`
- Control checkpoint: `cfa6ea5d93dcca9bf734fb85c67ba2c3b32e45d008c09a63d7ff04a222ac76eb`
- Geometry checkpoint: `e80471ed77a9c7816c67235d0c23e816f18409f0e20625a8f97188ff4ae711cd`
- 동결 verifier: `271b688ac62d0fc03a482fb60e5317211f1fde50c56db248d144296fdb906798`
- [완료 전 사전등록](fit_verifier_preregistration.json): `6f4c8902975b02cf7cf89f8055a2f0e6a4d48dfd86eca9c6b8f4429bce732ba6`

원본·frozen source·사전계획·실패 기록·루트 README는 변경하지 않았습니다.
