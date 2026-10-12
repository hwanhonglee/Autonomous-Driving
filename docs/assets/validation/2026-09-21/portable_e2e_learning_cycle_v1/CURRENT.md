# 9월 21일 — J causal-route 성능 회복 진단 완료

<!-- HH_260906 - Keep the latest decision focused on recovering the baseline before adding feature heads. -->

## 현재 판정

최신 J 실험은 PRO6000 물리 GPU 0에서 실제로 완료했지만
`FAIL_CAUSAL_ROUTE_RANKER_RECOVERY_SIGNAL`이다. metric gate `4/9`, geometry `3/3`,
TRAIN/DEV preservation `2/2`이며 J scorer는 승격·runtime export하지 않는다. TEST, GPU 1,
closed-loop CARLA, Autoware 제어, 실차, 신규 기능 head는 사용하지 않았다.

원격 payload 36개의 manifest 검증을 모두 통과했고 `failed.json`은 없었다. manifest 자체를
포함한 37개 파일, `11,024,629` byte를 로컬
`artifacts/training/2026-09-21/moving_causal_route_ranker_j_v1`로 exact 복사했다. 원격·로컬
경로/크기 및 모든 SHA-256이 일치하며 manifest SHA-256은
`cfd0be457e5fe3c8ab7ecbd74e0ffb79d1d237306f950db50a3acfcffc828b51`, summary SHA-256은
`f9c285bab037fa30e3aba91ebd115e7fb4092c1b9fbe6936dd179cfe76c1f0bc`다.

## 무엇을 학습했나

J는 F의 후보와 DRIVE/STOP mode를 고정하고 raw causal route를 candidate geometry에 직접
대응시킨 4-parameter shared cost scorer다.

- 입력: route polyline, 현재 speed/longitudinal acceleration, frozen candidate XY/speed
- 금지: target, label, 미래 ego, image, fused context, map/episode ID, F candidate ranking logit
- 학습: F-original DRIVE `1,232`행만; STOP `243`행은 loss exposure `0`
- 물리 route group: Town07 same-route `625`, C-track `289`, Town01 `318`
- 각 LOO fold 256 step + final 256 step, 총 row exposure `946,176`
- shared nonnegative feature: lateral, heading, speed, curvature
- DEV outcome은 final scorer를 고정한 뒤 한 phase에서만 집계

## 결과

| 범위 | 고정 상한 ADE / FDE / 속도 | J | 통과 |
| --- | --- | --- | --- |
| Town03 우회전 337 | 2.935314 / 7.793866 / 1.107615 | 2.994809 / 8.008273 / 1.111932 | 0/3 |
| Town03 직진 328 | 3.764740 / 11.027441 / 1.787750 | 4.148009 / 7.209370 / 1.475513 | 2/3 |
| combined 665 | 3.563246 / 10.353537 / 1.635022 | 3.563605 / 7.614228 / 1.291262 | 2/3 |

route conditioning으로 직진과 combined의 FDE·속도 MAE는 회복했다. 그러나 직진 ADE는
`+0.383269 m`, 우회전 ADE/FDE/속도는 각각 `+0.059495 m`, `+0.214406 m`,
`+0.004317 m/s`, combined ADE는 `+0.000360 m` 기준을 넘었다. 아주 작은 차이도 고정 gate
실패이며, 이미 여러 번 본 Town03 DEV에 맞춰 threshold를 바꾸지 않는다.

TRAIN/DEV 모두 mode change, fallback-logit mismatch, DRIVE-group-maximum mismatch가 `0`이다.
단, C-track LOO의 policy-applied hold DRIVE가 `0`이므로 3-fold 결과를 세 물리 route에서의
정책 일반화로 해석할 수 없다. stationary DEV가 봉인되어 stop guard도 아직
`NOT_EVALUABLE_NO_SEALED_STATIONARY_DEV_EPISODE`다.

## 다음 순서

1. 승인된 create-only 15 km/h capture 계약으로 TRAIN 역할 Town의 서로 다른 물리 straight
   route를 수집한다.
2. Town03 DEV와 Town04/Town15 TEST는 수집·학습에서 계속 제외한다.
3. C-track straight 후보 둘의 exact corridor section `7`개 중복을 route QA에서 제거하거나
   동일 group으로 묶는다.
4. J 구조, C/D 9-metric 상한, seed와 exposure를 고정한 data-only A/B를 수행한다.
5. metric `9/9`, geometry `3/3`, preservation `2/2`를 회복한 뒤 stationary 독립 DEV, 신규
   seed, 신규 물리 route confirmation을 진행한다.
6. 기존 약 210 m route는 30 km/h 계약에 필요한 `333.333 m`보다 짧으므로 30 km/h는 긴
   route가 준비된 뒤 진행하고, 60 km/h는 그 다음 별도 안전 계약으로 진행한다.
7. 이 성능 기준선을 회복한 뒤에만 장애물 회피·ACC·차선 변경 등의 기능 head를 하나씩 붙인다.

NuRec은 별도 real-to-sim 축이다. 실차 로그 기반 신경 장면은 domain 다양성에 기여할 수 있지만
현재 route ranker 실패를 자동으로 고치지 않는다. 기존 CARLA 0.9.15를 보존하고 성능 회복 뒤
별도 CARLA 0.9.16+ 환경의 공식 scene 파일럿과 실제 센서 로그 검증으로 진행한다. 현재는 어떤
패키지도 새로 설치하지 않았다.

[J 전체 결과](07_moving_causal_route_ranker_j/README.md) ·
[J 실패 분석](07_moving_causal_route_ranker_j/analysis/README.md) ·
[NuRec 검토](../../../../nurec-real2sim-feasibility-ko.md) ·
[cycle 목차](README.md)
