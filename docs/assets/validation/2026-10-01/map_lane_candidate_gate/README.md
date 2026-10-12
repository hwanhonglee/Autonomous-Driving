# 지도 차선 손실 후보: 현재 TRAIN 근거와 실행 관문

<!-- HH_261001 - Separate a possible diagnostic fit from unsupported quality certification or promotion. -->

**판정: GPU 재학습과 모델 교체 모두 지금은 NO-GO.** 기존 승인 자료로 `lane_marking` 손실 가중치만 바꾸는 *새로운 진단용* A/B의 계획은 만들 수 있다. 그러나 과거 W2 파일럿은 실패했고, 차선 성능 개선이나 독립 일반화 통과를 판단할 미사용 DEV/TEST 장소는 현재 **0곳**이다. 검증 분할·새 권한·동결 소스를 확보하기 전에는 이 후보를 실행 우선순위로 권하지 않는다. CTrack은 EVAL 전용이다.

## 현재 자료의 정확한 경계와 byte pin

<!-- HH_261001 - Keep the already-trained FALSE source distinct from the uncollected same-tick OFF/ON proposal. -->

최신 geometry의 연구 한정 TRAIN은 원본729 FALSE + 과거 동시촬영 Town01 p01/Town02 p00 TRUE480 + **과거 Town01 p02/p04 FALSE480**이다. 마지막480은 각 경로 240장의 `p02:frames/0000..0239`, `p04:frames/0000..0239` 전체로, 원래 0.25m 중복 위치를 제외하지 않는다. 동일 학습 풀에는 p02/p04 TRUE가 **없다**. [새 p02/p04 동일 틱 RGB OFF/ON 수집 계획](../map_paired_p02_p04/PLAN.md)은 별도 물리 주행·별도 dataset으로 아직 실행되지 않았으며, 옛 FALSE와 새 TRUE를 동일 프레임 쌍으로 연결할 수 없다. 새 자료의 `dataset_admitted=false`, `train_eligible=false`가 유지된다.

| 현재 remote 개인 TRAIN 입력 | SHA-256 / 크기 |
| --- | --- |
| 최초 p02/p04 후보 source manifest, 상태는 당시 미승인 그대로 | `3c89ab9516e032122728d01bd72b1f9e5c31b5e287253e292143aa67e06382a6` |
| 기존 p01/p00 reference package manifest | `32ed912505e0b215bfc54e563fcdba7553aa25412ba22d90b45bfa00339e93c8` |
| geometry 연구 한정 prepared manifest | `db54a65424b55bbd095373a3aecb43508037efa0f3d61b31354a14206371071d` |
| `original_train729.npz` | `d2d7b8b306fcba9672a229b4b1222b208e8d17fe500caf43755687612025e8f3`, 179,427,694B |
| `paired_train480.npz` (p01/p00 TRUE, p02/p04 아님) | `1e6c110cc20ba5dbba58cf1bf4ecebc1898da8172d18c279a5338cc539b5a116`, 265,179,122B |
| `new_false480.npz` (p02/p04 FALSE) | `f1fd23b786aa4dee7373054ed1580a4ad7a87715e05a62d2ec31b4ae35591f05`, remote read-only `stat` 116,751,125B |
| 고정 `training_schedule.npz` | `80659aa9eeecd3de590623a88f357a615dd39f74f268c756ee18ed46ff9ef5d9`, remote read-only `stat` 369,759B |
| 원래 context-B 부모 안전 NPZ | `ff5d7d03a43b859d21a0a3f6b01d815054d89d78e850730121f326e5b79c023e`, 922,682B |
| 학습 reference plan / 기존 geometry source bundle | `5731cbd35feff6e4a04b5e929279dbf553c327db6c70420f2a7126cc08487b68` / `20697b557b8f14a645b6e0ef36833184de7188b77a3e64c48496da4b6437ef5c` |

기존 geometry의 package·fit 허가는 **해당 control/geometry 한 쌍만** 대상으로 이미 소진됐다. 새로운 손실 후보에는 새 source/data/authority pin이 필요하다. 이전 Town01 DEV118, Town02 p02 holdout232, CTrack LEFT/STRAIGHT는 이미 연구에서 열어 본 장소이고 독립 DEV/TEST로 다시 이름 붙이지 않는다.

## 현재 근거: 왜 단순 재학습이 아닌가

| 근거 | 관측 | 해석의 한계 |
| --- | --- | --- |
| [CTrack 전체240 지도](../map_geometry_full_projection/README.md) | geometry 차선 셀 IoU LEFT **9.34%**, STRAIGHT **21.88%** | 차량용 HD-map 합격이 아니다. |
| [원본 픽셀 진단](../map_geometry_full_projection/lane_raw_pixel_attribution_v1/REPORT.md) | 채택 차선 증거가 없는 정답 셀의 같은 픽셀에서 lane `argmax`는 **0/1,048 LEFT, 0/801 STRAIGHT**; 그 픽셀의 road 승자 비율 **98.09% / 80.58%** | 분류 누락이 크지만 학습 데이터 부족·색상·CARLA 라벨 중 원인을 특정하지 못한다. |
| [지난 W2 파일럿](../../2026-09-29/map_lane_weight_pilot/README.md) | 옛 Town01 내부 holdout 한 짝에서 lane **95.258→95.378%, +0.120%p**, road −0.033%p; 실패 후 8짝 중단 | 옛 D1/구조/훈련 레시피의 결과이며 최신 geometry 모델의 W2 효과가 아니다. 당시 `+5%p` 기준은 baseline95.258%에서 최대 이득4.742%p라 애초 통과 불가능했다. |
| [최신 geometry known350](../map_unused_route_inventory/KNOWN_RESULTS_KO.md) | Town01 DEV lane **97.482%**, Town02 p02 holdout lane **98.535%** | 둘 다 이미 노출된 장소. 여기서 일률적 `+5%p`를 요구하면 가능한 최대 이득이 각각2.518/1.465%p뿐이다. |

따라서 지난 실패를 뒤집어 쓰거나 같은 `+5%p` 관문을 다른 고득점 장소에 그대로 적용하지 않는다. 새 손실의 영향은 **반드시 다른 이름의 사전 계획**에서 측정한다. CTrack 원래 LEFT owner 실패도 바꾸지 않는다.

## 읽기 전용 TRAIN 원본 QA

<!-- HH_261001 - Quantify only semantic-tag lane support; do not interpret 3x3 support as physical stripe width. -->

[실행기](../../../../../tools/e2e/audit_map_train_lane_support_20261001.py)(SHA-256 `724d74302d895cd2613de535d2b6c67897434aa975a741b26fffd648010ecf09`)는 승인된 geometry 연구의 p02/p04 원본 **480 sensor NPZ 전부**를 기존 manifest·연구 승인 SHA와 각 파일 SHA로 묶고 CARLA lane tag24를 다시 센다. DEV·CTrack·모델 추론·파일 쓰기는 없다. 차선 픽셀의 3×3 이웃 중 같은 차선 태그가 9개 모두 아닌 픽셀을 ‘완전한 3×3 내부가 아님’으로 집계했다.

| TRAIN 경로 | 차선 양성 프레임 | 차선 픽셀 | 완전한 3×3 내부가 아닌 픽셀 | 비율 |
| --- | ---: | ---: | ---: | ---: |
| p02 우회전 | 240/240 | 118,189 | 70,069 | **59.29%** |
| p04 좌회전 | 208/240 | 112,400 | 62,649 | **55.74%** |
| 합계 | 448/480 | 230,589 | 132,718 | **57.56%** |

이 수치는 원본 GT에 가는/경계 영향이 큰 부분이 많다는 **표본 지지량**이다. 실제 페인트 폭, 렌더 aliasing, 외부 CTrack 오류의 인과관계는 측정하지 않는다. 같은 GT가 이미 geometry 학습에 모두 포함됐으므로 ‘차선 라벨이 없어서 실패했다’는 단순 설명도 맞지 않는다. 3×3 정의는 [3개 합성 테스트](../../../../../learning/tests/test_audit_map_train_lane_support_20261001.py)로 검증했다.

```bash
CUDA_VISIBLE_DEVICES='' OPENBLAS_NUM_THREADS=1 python3 -B tools/workspace_python.py tools/e2e/audit_map_train_lane_support_20261001.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CUDA_VISIBLE_DEVICES='' python3 -B tools/workspace_python.py -m pytest -- -q -p no:cacheprovider learning/tests/test_audit_map_train_lane_support_20261001.py
```

실제 실행: QA **PASS**, 합성 테스트 **3 passed**, 파일 쓰기·GPU 사용 없음. 위 숫자는 stdout만 읽어 이 문서에 기록했다.

## 검증 자료가 생긴 뒤 준비할 *한 가지* 후보와 정지 조건

1. 먼저 과거 최적화에 사용되지 않은 장소의 독립 경로 최소 두 개를 별도 capture/QA하고 TRAIN과 분리해 위치·원본 해시·평가 스크립트를 학습 전에 고정한다. 현재 확보된 수는 **0/2**다. 새 p02/p04 paired 수집은 같은 Town01 기존 경로이므로 이 독립 두 경로를 대신하지 못한다. 그다음 새 연구 이름·자료 역할·출력 경로·입력/소스 SHA·GPU0 lease·동결 evaluation plan을 따로 등록한다. 기존 geometry fit 승인 JSON이나 지난 W2 파일럿 실행기를 재사용하지 않는다. 현재 원격 개인 자료는 제자리 읽고 로컬에는 큰 배열을 복제하지 않는다.
2. **한 요인만** 시험한다: 최신 geometry 연구와 같은 원래 context-B 부모, 원본729 FALSE+paired480 TRUE+geometry p02/p04 FALSE480, 같은 `10,000 step × batch8`, seed20261001, slot/crop/TRUE 광도 시퀀스, 모델·optimizer·loss 정규화로 두 fresh arm을 학습한다. 후보 arm의 class weight index3만 정확히 `1.4355568885803223 → 2.8711137771606445`(2×)로 변경한다. 다른 6개 weight·샘플/라벨/threshold는 고정한다. 두 final 저장·SHA 확인 **전에는** 어떤 DEV/EVAL도 열지 않는다. 이는 이전 W2 실패가 있어도 최신 레시피에서의 *진단* 가능성만 묻는 것이며 성공을 기대 결과로 적지 않는다.
3. 사전 고정한 전체240 CTrack LEFT/STRAIGHT **이미지 픽셀**과 알려진 Town01 DEV118·Town02 holdout232의 7×7 confusion, IoU, lane FN/FP, road와 다른 다섯 클래스를 한 번에 공개한다. 같은 저장 depth/pose·0.2m 누적을 두 arm에 동일 적용한 BEV LEFT/STRAIGHT 결과도 별도 분모로 공개한다. 유리한 프레임·threshold·checkpoint를 고르지 않는다. 이미 노출된 장면에서 후보가 CTrack 양 경로 lane IoU 각각 **+3%p 이상**, road IoU 각각 **−1%p 이내**가 아니면 **그 후보의 후속 학습을 중단**한다. 통과해도 내부 진단 신호일 뿐 모델 승격은 아니다.
4. 실질적 품질 인정은 1번에서 **학습 전에 지정하고 해시·경로·렌더 조건을 봉인한 독립 두 경로**에서만 판단한다. route별 lane IoU 오류 `1−IoU`를 기준으로 후보가 대조군보다 **10% 이상 감소**, road IoU는 각 경로 **−1%p 이내**, 전체 FP·다른 클래스는 모두 공개한다. 오류 감소는 고득점 IoU에서도 수학적으로 가능한 기준이다. 자료가 없거나 결과가 미달하면 승격하지 않는다. 동일 프레임의 FALSE/TRUE는 독립 두 장소로 세지 않는다.

**얇은 차선 우선 샘플링은 이번 한 요인 후보에 섞지 않는다.** GT 3×3 지지량은 확인됐지만, 샘플러를 바꾸면 노출 순서/분포까지 바뀌어 weight 효과와 분리되지 않는다. W2가 실패하거나 새 paired 자료 QA가 끝난 뒤, 별도 일정·동일 노출량·heldout 계약으로 시험한다.

## 비용·환경 및 현재 실행 여부

기존 geometry 두 arm 실제 GPU 학습은 **424.38초 +423.18초 ≈14.1분**이었다. **검증 자료·권한이 먼저 충족된 경우에만**, 같은 규모의 새 짝은 CPU 사전 검사·해시·평가·전송까지 **약20–45분 작업 창**을 예약한다(보장 시간 아님). 기존 local fit artifact 전체는 약**12MiB**였지만 안전한 원격 실행에는 raw/package 기존 참조와 압축 배열, 실패 영수증 여유를 위해 **remote 개인 경로 가용2GiB 이상**을 다시 확인한다. 원격 개인 경로의 read-only `df`는 이번 점검에 **3,127,477,161,984B** 가용이었다. 로컬에 원본480 복사본을 만들지 않는다. 현재 로컬 가용 공간은 2026-10-01 점검 때 약**5.17GB**로 별도 CARLA owner의6GiB 시작 조건보다 작다.

원격은 **pro6000 개인 `hwanhong/portable_e2e/venvs/py312`**, 물리 **GPU0 UUID**만, 기존 package만 사용한다. GPU1·공용 conda/base·설치·다른 작업 종료는 금지한다. 현재 이 제안의 runner/source freeze/새 fit 승인·학습은 **없다**. 그래서 실행 가능한 새 학습 명령도 아직 없다. 먼저 정확한 신규 runner/CPU preflight/동결 계획을 만들고 검증해야 한다. 이번 작업은 QA와 정지 판단까지만 수행했다.
