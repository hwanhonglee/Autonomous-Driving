# 미사용 지도 원본 감사 — Town01 두 경로

## 현재 요약: 한정 학습 완료·known350 독립 검증 PASS

<!-- HH_261001 - Keep the original inventory narrative historical and report completed evidence separately. -->

Parent의 별도 승인으로 **control/geometry 각 10,000 step 학습이 완료**되었고, import한 final/source/노출 영수증과 known350의 **161,280,000픽셀 독립 재집계가 PASS**다. [한국어 결과와 퇴행 항목](KNOWN_RESULTS_KO.md) · [독립 검증 영수증](known_split_independent_recount.json). Town01 DEV는 7클래스 IoU가 모두 올랐지만 Town02 holdout의 road·sidewalk·pole·other는 하락했다. 승격이나 독립 검증 통과를 뜻하지 않는다. CTrack 2종과 기존 TRAIN 2종의 별도 네 시나리오 평가는 다른 분모로 유지한다.

**아래 최초 감사·제안 내용은 당시의 미승인 상태를 보존한 역사적 기록이다.** 후보 manifest의 당시 권한, 과거 소형 표지판 실패, frozen plan/영수증은 수정하지 않는다. 실제 승인·완료 경과는 아래 후속 절과 현재 실행 상태를 따른다.

## 최초 감사 당시의 판단

<!-- HH_261001 - Separate valid sensor captures, prior task rejection, and prospective study admission. -->

**활용 가능한 미사용 원본은 있다: Town01 p02 우회전·p04 좌회전의 480장. 아직 TRAIN 편입이나 학습은 승인하지 않았다.** 두 수집은 owner·lifecycle·원본 재생에 성공했고, 과거 거부 이유는 **1–4픽셀 소형 표지판 0장**이었다. 그 실패를 유지하면서, 도로·차선 중심의 새 연구 목적만 별도 검토할 수 있다. 새 Town·날씨·TRUE-render 자료는 아니다.

## 현재 사용·보호 범위

| 로컬 자료 | 현재 상태 / 이번 결정 |
| --- | --- |
| Town01 직진 114 + p03 187 + p01 196, Town02 p00 232 | 현재 original729 FALSE에 이미 사용. 원본 900장 중 과거 0.25m 규칙으로 제외된 171장은 새 경로 다양성이 아니다. |
| Town01 p01·Town02 p00 paired v2 각 240 | 같은 프레임 FALSE/TRUE 두 RGB와 공통 라벨. 현재 TRUE480에 전량 사용. v1 실패 수집은 제외. |
| **Town01 p02·p04 각 240** | 미사용 원본. 이번 후보 참조 manifest는 **전체 480장**, 편입·학습 권한은 false. |
| Town01 DEV 118 / 해당 전체 경로·초기 재촬영 | 계속 DEV 보호. moving_turn_clear_v2 240, moving_turn_v1 180, turn_v1 32를 새 TRAIN으로 재분류하지 않음. |
| Town02 p02 232 / 전체 240 경로 | 기존 모델에 노출된 내부 route holdout. 현재 TRAIN에 다시 넣지 않음. 다른 Town 좌표를 Town01 좌표와 직접 거리 비교하지 않음. |
| Town01 straight_v1 정지 32 | 기존 직진 TRAIN과 같은 경로의 짧은 센서 프로브. 새로운 다양성 자료로 세지 않음. |
| Town03, Town05_Opt | 폴더 이름이 training이어도 현재 DEV/분할 충돌 제한을 우선. 새 TRAIN 편입 불가. |
| CTrack | TRUE480 외부 진단 EVAL 전용. 잘못 라벨링된 이전 packaged 수집도 제외. 과거 V2 capture-only 권한은 TRAIN 승인 아님. |
| Town04·06·07·10HD_Opt·15 | 예약/보호 사이트. 이번에 해당 원본을 열지 않음. |
| Town12·13·기타 주행 수집 | 현재 허용된 미사용 지도 TRAIN semantic 쌍 없음. RGB 주행 영상만으로 semantic 학습쌍을 만들지 않음. Town13 실패 수집 제외. |

검색 범위는 로컬 `storage/experiments/{training,validation}`와 `storage/datasets/prepared`의 capture/manifest 파일명이며, 허용 Town01/Town02 mapping summary 14개를 대조했다. native114 → expanded301 → expanded497 캐시는 같은 원본의 누적이다. road_lane D1의 Town02 p02, 과거 Town03 패키지는 현재 보호 규칙을 우회하는 새 데이터가 아니다. 승인된 독립 DEV는 여전히 0곳이다.

## 전량 감사 결과

| 항목 | p02 우회전 | p04 좌회전 |
| --- | ---: | ---: |
| 전체 / 이번 제안 프레임 | 240 / **240** | 240 / **240** |
| 과거 0.25m 보존 프레임 | 197 | 207 |
| 손상·누락으로 제외할 프레임 | 0 | 0 |
| road 픽셀 | 23,248,353 | 22,666,951 |
| lane_marking 픽셀 / 양성 프레임 | 118,189 / 240 | 112,400 / 208 |
| sidewalk 픽셀 | 8,248,020 | 8,632,148 |
| traffic_light_fixture 픽셀 | 80,561 | 64,930 |
| traffic_sign 픽셀 | 1,691 | 944 |
| pole 픽셀 | 321,121 | 172,115 |
| other 픽셀 | 23,278,065 | 23,646,512 |
| 주행 거리 | 79.492m | 83.989m |
| 기존 Town01 TRAIN 최소 위치 거리 | 73.515m | 73.456m |
| 고정 Town01 DEV 최소 위치 거리 | **128.360m** | **176.680m** |
| paired Town01 p01 최소 위치 거리 | 242.673m | 250.163m |

두 경로의 480 RGB PNG·480 sensor NPZ를 모두 읽고 해시·크기·순서·0.1s 간격을 검사했다. 각 NPZ에는 원본 CARLA semantic/depth `360×640×4 uint8`, 유한 `4×4 world_from_camera`가 있다. 깊이 0 픽셀은 0이며 최대 깊이는 배경/하늘을 포함한다. 깊이·pose는 모델 입력이 아니라 기하/검증 증거다. 두 owner 종료 코드 0, 충돌 0, live/replay 일치, 13개씩의 보관 source와 세 lifecycle PASS를 재확인했다.

과거 제외된 p02 43장·p04 33장은 **이전 보존 위치에서 이동 0.25m 미만**이라는 규칙의 결과다. 초기 가속 구간과 마지막 감속·정지 구간이며, 손상·잘못된 라벨 판정이 아니다. 전체 인덱스를 manifest에 남겼다. p04의 차선 없는 32장도 유효한 회전/무도색 시야로 그대로 포함한다. 전량 보존으로 시간 상관과 정지 반복이 커지는 한계도 숨기지 않는다.

두 후보 간 최소 위치 거리는 0.017m이며, p02 84장·p04 59장은 상대 경로 위치에서 1m 이내다. **독립 두 사이트/완전 독립 두 경로로 세지 않는다.** 원래729 및 DEV/holdout에 대한 PNG 파일 해시 중복은 0, paired480 양 RGB에 대한 decoded HWC uint8 픽셀 해시 중복도 0이다. 파일/위치 비중복은 전체 화각의 객체 비중복 증명이 아니다.

## 고정 전체 화각 검토

각 PNG는 0·119·239 순서, 위 RGB·아래 oracle 7클래스이며 패널별 **640×360 원본 해상도**다. [p02 전체 화각](audit_v1/p02_fixed_rgb_oracle.png) · [p04 전체 화각](audit_v1/p04_fixed_rgb_oracle.png).

여섯 native RGB와 여섯 oracle 패널을 확인했다. 도로/보도 경계, 황색선, p04 회전 중 도로와 흰색 연석·가드레일 분리가 합리적으로 정합한다. 눈에 보이는 UE 경고문은 없다. **하얀 하늘·어두운 수목/건물의 FALSE-post 외관은 그대로**이며, TRUE-render 정상 조명 자료로 재명명하지 않는다. ClearNoon·postprocess=false는 해시 고정된 당시 collector 설정에서 확인했고, 원래 수집은 개별 blueprint attribute 영수증을 저장하지 않았다는 제한도 남긴다. 고정 여섯 장 검토는 480장 전수 육안 승인이 아니다.

추가 parent 검토에서 p04 frame119 장벽 주변 oracle 보라색 영역을 지적했다. [고정 좌표 원본 검사](barrier_tag_check.json)에서 흰 수평 장벽 `(320,198)/(450,222)`는 **tag28→other**, 어두운 받침/가장자리 `(250,183)/(450,256)/(630,241)/(630,275)`는 **tag2→sidewalk**다. 비교 도로 `(400,310)`는 tag1→road, 수목 `(430,140)`는 tag9→other다. 로컬 CARLA `ObjectLabel.h`에서도 GuardRail=28이다. 일부 비수평 표면에 붙은 tag2는 mesh-group ontology 한계일 수 있으며 완벽한 GT/지면 마스크가 아니다. **재라벨링·프레임 제외·투영 필터 변경 없이 그대로 보존**한다.

## 권장하는 다음 한 가지

새 head·더 많은 step 대신 **같은 모델/학습법에서 기존 replay 대 미사용 geometry-source 480장**만 비교한다. 현재 TRUE TRAIN 재현은 road 약99.4%인데 CTrack STRAIGHT는41.0%이며, 실제 도로의43.37%가 여전히 보도로 간다. 광도 증강은 일부 개선됐지만 알려진 Town02 차선 보존은 퇴행했다. 이 증거는 추가 최적화보다 학습 장면의 범위를 확인할 이유가 된다. 다만 이번 두 경로는 FALSE Town01이므로 외부 TRUE/generalization 개선은 가설이며 보장하지 않는다.

[한정 연구 제안](PROPOSAL.md)은 기존729+TRUE480을 양팔 공통으로 두고, 480개의 같은 추가 slot에서 control은 **기존 Town01 FALSE497 중 고정480 replay**, candidate는 이 두 경로의 전체480을 사용한다. Town·postprocess까지 맞춘다. 같은 원래 B 부모·구조·class weight·10,000 step·80,000 exposure·crop·TRUE480 광도 정책을 고정한다. 추가 slot 수·기존/TRUE exposure 감소는 양팔 같고 공개하며, 이전 photometric run을 동등 control로 재사용하지 않는다. [정확한 replay 인덱스](reference_plan.json)와 [remote-only 포장 계획](REMOTE_PACKAGE.md)도 준비했다. 별도 연구 TRAIN registry 승인과 parent 검토 전 **데이터 편입·fit 없음**.

## 증거 및 재현

- [후보 참조 manifest](audit_v1/candidate_manifest.json): `3c89ab9516e032122728d01bd72b1f9e5c31b5e287253e292143aa67e06382a6`.
- [독립 원본 재집계](audit_v1/independent_recount.json): 전체 **110,592,000픽셀**, 원본 해시 불변. 기존 원본·registry·실패 기록은 수정하지 않음.
- `tools/e2e/inventory_map_unused_routes_20261001.py`: 후보만 생성, 기본 fresh output, 기존 폴더 덮어쓰기 거부. 빌더 `d07167285b947c850e37f21b92c74918fae8fe00e8e3351ac9f144d3f316dc0c`.
- `learning/tests/test_map_unused_routes_20261001.py`와 `test_map_route_geometry_remote_20261001.py`: 합계 **12 PASS**. 실패 owner·보호 사이트·로컬 배열 생성·미승인 포장 거부, 0.25m 규칙, 7클래스 ABI, 실제 보관 광도 함수와 새 trace 일치, transfer 대상 480 NPZ만 포함하는지 검사.

새 NPZ cache·촬영·추론·GPU·설치 없이 참조 JSON과 두 PNG만 작성했다. 전체 새 코드·문서·증거도 2MiB 미만이다. HD-map 완성·주행·독립 검증·모델 승격을 주장하지 않는다.

## 후속: 한정 역할 승인과 remote 포장 완료

<!-- HH_261001 - Append the real package authorization transition without revising the earlier inventory verdict. -->

Parent가 원본480·고정 replay·DEV 분리·장벽 taxonomy 한계를 검토한 뒤 [한정 study TRAIN/remote package 권한](package_authorization.json)을 생성했다(SHA `844bbc685eb0e49c427687d53432d622d3d8123e519e870423c460375bdfe43e`). 이어 parent가 개인 remote에서 CPU로만 포장한 [실제 manifest](remote_package_manifest.json)의 SHA는 `db54a65424b55bbd095373a3aecb43508037efa0f3d61b31354a14206371071d`다. 로컬에는 배열을 만들지 않았고, 원본 candidate manifest의 당시 `train_eligible=false`와 기존 거부 기록도 고치지 않았다. **Package 권한의 fit_authorized는 여전히 false**이며 별도 fit 승인 전 학습은 없다.

후속 [학습 프로토콜](TRAINING_PLAN.md)은 실제 선행 class-weight 값을 동일하게 고정한다. 이전 “원래 Town01 weights” 표현은 [명시적 정정](WEIGHT_PROVENANCE_CORRECTION.md)으로 바로잡았으며, frozen plan·packager·과거 fit의 값을 바꾸지 않았다. 새 source-only trainer/test archive를 포함한 이 후속 작업은 5MiB 미만으로 유지한다.

## 현재 실행 상태: 승인된 한 쌍 완료·known350 독립 검증 PASS

<!-- HH_261001 - Report the completed authorized pair while preserving all frozen plans and receipts. -->

Parent가 frozen trainer·contract·plan·실제 weight 정정을 검토했고, remote CPU preflight는 전체480/배열/고정 schedule 검사 **PASS**였다. 로컬 CPU 테스트도 **29/29 PASS**다. [별도 fit 승인](fit_authorization.json) SHA `8e4e586d8d80eb0bb3c645ad01975954ba34dfb3f29cdaac07fc5d610da1fa38`로 **control→geometry 10,000-step 한 쌍만** 승인되었다. Parent의 GPU0 session27625가 종료 코드0으로 완료되었고, final 산출물을 지정한 로컬 `fit/`에 import했다. 최종 summary SHA는 `caa1a60bcff8179b80de0a5861aa2ab0497d838638fbf80a3f4790e8074e11ef`다. 두 팔의 실제 학습 시간은 각각424.38/423.18초이며 이 준비 agent는 추가 GPU/SSH/촬영/설치를 실행하지 않았다.

고정 source bundle SHA는 `20697b557b8f14a645b6e0ef36833184de7188b77a3e64c48496da4b6437ef5c`. [실행·영수증·실패 처리 handoff](TRAINING_HANDOFF.md)를 보관한다. 두 final 저장 이전 DEV 열람 없음, CTrack EVAL 전용, 자동 승격 없음은 그대로다. Archive 안 문서의 대소문자 링크는 원본 문서 이름과 archive의 `weights_correction.md` 이름이 달라 직접 열어야 하지만 문서 bytes·학습 소스·동결 해시는 수정하지 않았다.

실행 중 후보를 열지 않고 [독립 final-fit/known350 재집계 프로토콜](FIT_VERIFIER_PLAN.md)을 동결했고, 추가22개 검증기 테스트가 PASS했다. 완료·import 후 parent가 제공한 최종 summary SHA로 전체 **161,280,000픽셀**, 36개 artifact 해시, source9, 팔별52개 float32 tensor, 100개 history row/10,000 step, 실제 sampling/crop/광도 trace와 원본 보존 노출을 독립 대조하여 **PASS_DIAGNOSTIC_NOT_PROMOTED**를 기록했다. [독립 영수증](known_split_independent_recount.json) SHA `760861c1f4327f98c8a2c3a66b0a22f666a85100ff47bd2e026de69429a66558`. 새 추론·GPU·pickle 로드 없이 저장 예측만 재집계했고, 알려진 split의 개선과 퇴행은 [결과](KNOWN_RESULTS_KO.md)에 모두 남겼다. 별도 네 시나리오 평가와 분모를 합치지 않는다.
