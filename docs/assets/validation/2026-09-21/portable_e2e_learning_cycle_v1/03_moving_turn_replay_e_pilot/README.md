<!-- HH_260906 - Document the sealed single-seed E replay result, recovery boundary, gate outcome, and next balanced-replay proposal. -->

# E 회전 cohort replay 단일 seed 진단

결론은 **E 미승격**이다. 전체 VAL과 직진 수치는 C보다 좋아졌지만, 사전 고정한
Town03 우회전 gate 3개를 모두 실패했다. 따라서 unseen 3-seed 확인 실험도 아직
승인하지 않으며, 모델 export·promotion·배포 근거로 사용할 수 없다.

## 무엇을 실행했나

E는 seed `20260906`으로 새로 초기화한 offline 학습이다. TRAIN 1,475개를 매 logical
epoch마다 한 번 노출하고, `max(abs(target_yaw_rad)) > 0.2`인 327개를 한 번 더 replay했다.

| 항목 | 고정 실행값 |
| --- | ---: |
| logical epoch | 20 |
| optimizer step | 9,020 |
| batch/epoch | 451 |
| base exposure | 29,500 |
| replay exposure | 6,540 |
| 전체 exposure | 36,040 |

실행 schedule 36,040행과 실제 history 9,020행은 일치한다. TEST는 읽거나 사용하지
않았으며 평가 대상은 고정 VAL 665개(Town03 우회전 337, 직진 328)다.

## 원본 실패와 별도 recovery

원본 runner의 4단계 가운데 `train`, `evaluate`, `audit` 3단계는 완료됐고 마지막
`subgroup`만 실패했다. checkpoint의 `train_config`에는 Python tuple이, JSON plan에는
동일 내용의 list가 들어 있었는데 evaluator가 정규화 없이 직접 비교해
`checkpoint replay contract/accounting differs`를 발생시킨 **자료형 비교 버그**다.

원본 실패 기록은 수정하거나 덮어쓰지 않았다. 별도 봉인한 subgroup-only recovery에서
tuple/list를 JSON 호환 표현으로 정규화한 evaluator만 사용했고, 학습·전체 평가·audit는
다시 실행하지 않았다. recovery 상태는 `COMPLETE_NOT_PROMOTED`이며 post-run 검증은
원본 3/4 실패 보존, 원본 tree 불변, checkpoint·dataset·episode binding을 모두 확인했다.

## C / E 고정 VAL 지표

낮을수록 좋다. `Δ`와 변화율은 `E - C`이며, loss는 무차원, ADE/FDE는 m, 속도 MAE는
m/s다. C는 기존 TRAIN 1,147개·7,380-step reference, E는 위의 9,020-step replay 모델이다.

| 구간 | 지표 | C reference | E replay | Δ (E-C) | C 대비 |
| --- | --- | ---: | ---: | ---: | ---: |
| 전체 VAL 665 | loss | 5.370674 | 3.823660 | -1.547014 | -28.80% |
| 전체 VAL 665 | ADE @ 6.4 s | 5.478203 | 4.551741 | -0.926462 | -16.91% |
| 전체 VAL 665 | FDE @ 6.4 s | 13.481566 | 11.997437 | -1.484129 | -11.01% |
| 전체 VAL 665 | 속도 MAE @ 6.4 s | 2.065069 | 1.841887 | -0.223182 | -10.81% |
| Town03 우회전 337 | loss | 3.612647 | 4.676562 | +1.063915 | +29.45% |
| Town03 우회전 337 | ADE @ 6.4 s | 2.935314 | 3.523328 | +0.588013 | +20.03% |
| Town03 우회전 337 | FDE @ 6.4 s | 7.793866 | 8.968759 | +1.174893 | +15.07% |
| Town03 우회전 337 | 속도 MAE @ 6.4 s | 1.107615 | 1.224754 | +0.117138 | +10.58% |
| Town03 직진 328 | loss | 7.176940 | 2.947355 | -4.229585 | -58.93% |
| Town03 직진 328 | ADE @ 6.4 s | 8.090865 | 5.608372 | -2.482493 | -30.68% |
| Town03 직진 328 | FDE @ 6.4 s | 19.325330 | 15.109218 | -4.216112 | -21.82% |
| Town03 직진 328 | 속도 MAE @ 6.4 s | 3.048795 | 2.475954 | -0.572841 | -18.79% |

원시 subgroup 자료: [C reference](metrics/C_reference/subgroup_summary.json) ·
[E turn replay recovery](metrics/E_turn_replay/subgroup_summary.json)

## 사전 고정 gate 판정

| gate | E 상한/최소값 | E 관측값 | 판정 |
| --- | ---: | ---: | --- |
| 우회전 ADE @ 6.4 s | ≤ 2.935314 m | 3.523328 m | **FAIL** |
| 우회전 FDE @ 6.4 s | ≤ 7.793866 m | 8.968759 m | **FAIL** |
| 우회전 속도 MAE @ 6.4 s | ≤ 1.162996 m/s | 1.224754 m/s | **FAIL** |
| 직진 ADE @ 6.4 s | ≤ 7.281779 m | 5.608372 m | **PASS** |
| 직진 FDE @ 6.4 s | ≤ 17.392797 m | 15.109218 m | **PASS** |
| geometry selected | ≥ 664/665 | 664/665 | **PASS** |
| geometry any candidate | ≥ 664/665 | 664/665 | **PASS** |
| geometry all candidates | ≥ 664/665 | 664/665 | **PASS** |

즉, 우회전은 **3 FAIL**, 직진은 **2 PASS**, geometry는 **PASS**지만 전체 gate는
`FAIL_SINGLE_SEED_DIAGNOSTIC`이다. E는 미승격이고 seeds `20260907/08/09` 확인도
승인되지 않았다.

## 왜 회전 replay만으로 우회전이 개선되지 않았나

327개 high-curvature membership은 방향을 균형화하지 않고 원본 비율을 그대로
복제했다. 실제 구성은 **left 254 : right 73**이므로 왼쪽이 77.68%이고, 오른쪽은
왼쪽보다 매 epoch **181 exposure 부족**하다. 이것은 replay가 우회전 성능을 직접
악화했다는 인과 증명은 아니지만, “회전 표본을 늘리면 우회전도 보호된다”는 가정이
성립하지 않은 명확한 sampling 설계 원인이다.

다음 F 후보는 전체 방향 노출의 균형을 사전 고정한다.

- base 1,475개는 epoch마다 한 번 유지한다. 이 base에 이미 left 254개와 right 73개가
  포함되어 있다.
- left는 replay하지 않고, right 부족분 `254 - 73 = 181`개/epoch만 replay한다.
- right 73개 모두에 2회씩 더 배정하고 남은 35회는 epoch마다 deterministic하게
  회전시켜 특정 표본 고정을 피한다. 결과적인 방향 노출은 left 254 : right 254다.
- 그러면 전체는 `1,475 + 181 = 1,656` exposure/epoch다. 같은 20 epoch라면 base
  29,500 + replay 3,620 = 총 33,120 exposure, batch 4 기준 8,280 step이다.
- F seed `20260906`이 동일 gate를 먼저 통과할 때만 unseen 3-seed 확인을 검토한다.

## 궤적 그림

아래 그림은 **offline fixed-DEV trajectory render**다. ego는 화면 하단 중앙, 회색은
route, 초록색은 expert trajectory, 굵은 색 선은 selected trajectory다.

- [Town03 우회전 C/E 6쌍 갤러리](galleries/town03_right_turn_dev.md)
- [Town03 직진 C/E 6쌍 갤러리](galleries/town03_straight_dev.md)

| 우회전 C reference | 우회전 E replay |
| --- | --- |
| ![C right-turn representative](visuals/town03_right_turn_dev/C_reference/0003_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000003.png) | ![E right-turn representative](visuals/town03_right_turn_dev/E_turn_replay/0003_carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9_000003.png) |

| 직진 C reference | 직진 E replay |
| --- | --- |
| ![C straight representative](visuals/town03_straight_dev/C_reference/0003_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000003.png) | ![E straight representative](visuals/town03_straight_dev/E_turn_replay/0003_carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320_000003.png) |

## 증거와 범위 제한

- [독립 C/E 비교와 gate](evidence/comparison_C_E.json)
- [post-run 검증](evidence/postrun_verification.json)
- [전송·inventory 검증](evidence/transfer_verification.json)
- [원본 3/4 실패 기록](evidence/failed.json)
- [별도 subgroup recovery 요약](evidence/recovery_summary.json)
- [stage 전체 SHA-256](SHA256SUMS)

이 stage는 CARLA/Autoware 라이브 주행 화면, RViz 전체화면, GIF, closed-loop 제어
평가가 아니다. all-Town 검증, 자율주행 30개 기능 검증, 실제 차량 안전성·구동 가능성도
입증하지 않는다. 모델 export, runtime 승격, 배포, 실차 제어는 모두 승인되지 않았다.
