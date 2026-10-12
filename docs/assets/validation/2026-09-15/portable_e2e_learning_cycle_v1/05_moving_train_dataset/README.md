<!-- HH_260906 - Record actual qualified TRAIN conversion and the authenticated remote handoff as successors to the immutable raw diagnostic, without model or vehicle approval. -->
# Moving pilot → Common10 TRAIN 328개: 실제 변환 완료

**로컬 결과: `QUALIFIED_COMMON10_PLANNING_CONVERSION` / Common10 planning validation `PASS`.**

2026-09-15 Town07 직진 moving pilot의 사전 고정 구간을 실제 Common10 TRAIN 데이터셋으로 변환했다. 준비된 TRAIN example은 **328개**, 독립 episode는 **1개**, 선택 시각 범위는 **32.700000487초**다. 모델 학습을 실행하거나 모델·차량 제어를 승인한 결과는 아니다.

**원격 수신도 완료:** 2026-09-15 **11:48:42 UTC / 20:48:42 KST**, 개인 prepared 경로로 이동한 뒤 1,980개 전체 파일·Common10 planning·TRAIN loader를 CPU에서 다시 검증했다. 상태는 `TRANSFERRED_AND_CPU_VERIFIED_NOT_TRAINED`다. **전송 완료와 새 데이터로 모델을 학습한 것은 다르며, 후자는 아직 실행하지 않았다.**

## 04와 무엇이 달라졌나?

[04 실제 수집·원본 진단](../04_moving_data_pilot/README.md)은 당시 판정 `DIAGNOSED_NOT_ADMITTED`와 원본 `PENDING_QA`를 그대로 보존한다. 이번 05는 그 이후 **명시적 moving-task qualification → 새 Common10 변환 → validation/loader/원본 재구성 검사**가 실제 완료된 별도 증거다. 04의 문구·원본·봉인은 소급 수정하지 않았다.

| 단계 | 실제 상태 | 의미 |
|---|---|---|
| 새 BasicAgent moving capture | complete, `task_complete=true` | 정해진 이동 구간과 future support 기록 완료 |
| 원본 04 진단 | `DIAGNOSED_NOT_ADMITTED` | 선택 구간의 필요조건 통과; 당시 데이터셋 편입 전 |
| 첫 변환 시도 | `REJECTED_BEFORE_DATASET_CREATION` | world-space 원본 route의 provenance 연결 미지원으로 dataset 생성 전 거부 |
| v2 명시적 qualification receipt | `QUALIFIED_FOR_COMMON10_CONVERSION` | 이 사례의 TRAIN-only staged conversion을 허용하는 source/input pin |
| 실제 변환 결과 | `QUALIFIED_COMMON10_PLANNING_CONVERSION` | Common10 validation·TRAIN loader·native 재구성까지 확인한 새 데이터셋 |
| 원격 수신·모델 학습 | **prepared 수신·재검증 PASS / 학습 사용 미실행** | 로컬 변환 성공과 별개로 수신 증거까지 확보 |

원본 catalog 목표는 여전히 `goal_reached=false`다. 목표 정지 대신 이동 과제의 종료 규칙을 명시적으로 사용했으며, 원본 goal·pose·속도·라벨·실패를 성공으로 고치지 않았다. BasicAgent expert 데이터이지, 학습 모델이나 현재 Autoware가 주행해 만든 성능 증거가 아니다.

## 실제 생성된 데이터

| 항목 | 결과 |
|---|---|
| dataset ID | `carla-town07-moving-pilot-20260915-v1` |
| 로컬 별도 데이터셋 | `artifacts/training/2026-09-15/moving_cohort_v1/prepared_moving_train_v1` |
| TRAIN / VAL / TEST | **328 / 0 / 0 examples** |
| episode / 맵 / 경로 | 1 / Town07 / `town07_straight_s0000_p00` 직진 |
| 환경·속도 | ClearNoon, seed **0**, 목표 **15 km/h** |
| 시간 범위 | 32.700000487 s, 실제 rate 9.999999851 Hz |
| 카메라 정합 | 6대, bundle coverage 100%, max skew 0 ms, p99/max gap 100.000002 ms |
| 미래 정답 | 328×64 = **20,992 point**, 유효 future 100%, native ego/navigation/target 재구성 일치 |
| 실제 데이터 크기 | **1,980 files / 213,747,472 bytes**, symlink 없음 |
| 준비된 영상 | 328×6 = **1,968 JPEG**, 원본에서 독립 복사 |
| 원본 보존 | 575 camera anchors / 3,450 JPEG hash 보존, 선택 밖 247 anchors context-only |

이는 **새 seed 0 pipeline pilot**이며 cohort planner의 seed 101 슬롯 완료가 아니다. 기존 duration campaign은 byte-frozen **TRAIN 1,147 / VAL 337**로 계속되며, 이 328개를 중간에 끼워 넣지 않았다. 따라서 현재 학습이 1,475개 TRAIN으로 수행됐다고 쓰면 안 된다. 새 데이터셋에는 독립 VAL·TEST가 없고, 여러 맵·회전·정지·날씨 다양성이 확보됐다는 뜻도 아니다.

## 무엇을 통과했고 무엇은 아직 아닌가?

[validation_report.json](validation_report.json)은 schema, planning 계약, image payload SHA, native source frame/manifest 연결, camera timestamp, causal route 재구성, offline 1 ms bundle readiness를 PASS로 보고한다. [moving_conversion_qualification.json](moving_conversion_qualification.json)은 별도로 다음을 기록한다.

- TRAIN loader 328개 성공, VAL·TEST 0개.
- native ego·navigation·20,992 future target의 정확한 재구성.
- 선택 구간의 기존 9개 검사 모두 violating anchor **0/328**: speed, speed accel/decel, XY speed/accel/decel/curvature/lateral acceleration, yaw envelope.
- 3,450개 원본 이미지 hash 보존과 1,968개 선택 영상의 독립 복사.
- 원본 `PENDING_QA` metadata, goal false, 원본 route bytes 및 전체 raw 실패 기록 불변.

공통 validator 자체의 `raw_source_artifact_content`와 `image_pixel_decode`는 **`NOT_RUN`**이며, `trajectory_geometry_recomputed_from_source_state=false`, `runtime_execution_tested=false`다. 이를 PASS로 고쳐 쓰지 않았다. 원본 content·픽셀 decode·선택 future 검사는 [별도 04 진단과 공개 검산](../04_moving_data_pilot/README.md), native→prepared target 재구성은 이번 qualification 결과로 구분한다. 검사 주체와 범위가 다르다.

전체 raw full-64-point 510개 창의 위반 anchor는 여전히 speed acceleration **40**, speed deceleration **63**, XY acceleration **38**, XY deceleration **63**, XY curvature **97**이다. 창이 겹치므로 이 수를 독립 사건처럼 더하지 않는다. 새 moving 과제의 사전 공간 규칙과 기존 QA를 통과한 선택 구간만 변환했으며, 원본 launch/cleanup 실패 삭제·사후 valid-mask 변경·과거 미승인 episode 재승인은 하지 않았다.

새 데이터의 planning 변환이 끝났다는 의미이지 학습 실행·성능 채택·release 또는 차량 제어 승인은 아니다. `training_run_performed=false`, `model_or_vehicle_release_approved=false`, `manual_release_review_required=true`를 그대로 보존한다. 데이터 workflow의 qualification과 차량/model release 판단을 혼동하지 않는다.

## 첫 실패와 v2 수정의 정확한 범위

[conversion_attempt_01.json](conversion_attempt_01.json)은 첫 실제 시도의 원본이다: exit 1, `native route lacks coordinate_alignment provenance`, dataset 생성 0, 학습 데이터 추가 0. 첫 receipt SHA는 `41c5ad6c854f36dac2eac83e3160cc60a59201eb64f2df2f6a6ee9c3b1856514`로 남겨 두었다.

v2는 이 새 moving 과제에 한해 **authenticated owner route path와 원본 catalog bytes**를 명시적으로 연결했다. 원본 route는 이미 world-space였으며, 존재하지 않았던 좌표 변환 증거를 합성하지 않았다. `coordinate_alignment_synthesized=false`, `native_route_alignment_transform_performed=false`, `native_route_bytes_unchanged=true`다. sample별 기존 map→base navigation projection은 유지했고 일반·과거 수집의 기본 coordinate-alignment 요구를 제거하지 않았다.

## 버전·SHA·공개 경로 보호

v2 qualification receipt SHA256는 **`a6d56dcac17e4e9aaeb2306060be33c21f66c3124078dad7ace9594159a313a7`**다. 약 683 kB의 전체 receipt와 큰 journals는 private 원본에 보존하고 여기 중복하지 않는다.

episode provenance의 base git commit은 `3cb676acbd2be4511be156d43ec002c554b438f3`이다. **그 commit만 checkout하면 이번 converter와 동일해진다는 뜻이 아니다.** 실행 adapter는 당시 uncommitted 수정본이고 qualifier는 새 소스였다. 정확한 실행 버전은 다음 SHA와 `conversion_v2_source_archive`의 **11개 source pin**으로 고정했다. [publication_provenance.json](publication_provenance.json)에 전체 경로·SHA·검산 결과가 있다.

| 소스 | 실제 실행 SHA256 |
|---|---|
| `scripts/e2e/prepare_carla_common10_dataset.py` | `12bf63832783ed2eb1e868d2ee30b5832d22b8bf46cd31fbdfcc29b8318e7a0c` |
| `scripts/e2e/qualify_carla_moving_common10.py` | `520acdb96f133d062f5e4391f66270be6ac4c449758718cfd2c6ba0819470ffe` |

source archive에는 위 2개와 `carla_moving_capture.py`, `audit_portable_target_feasibility.py`, Common10 contract JSON, `portable_e2e/__init__.py`, `contract.py`, `dataset.py`, `model.py`, `runtime_contract.py`, `stop_primitive_research.py`가 있다. 실제 모델 source나 loss를 이번 공개 작업에서 수정하지 않았다.

| 식별자 | SHA256 |
|---|---|
| 원본·공개본 동일 dataset manifest | `0cd658a8406ca3785844ab5cd0b44ac28d24466d1f3bc428b24cc541ce32db4c` |
| private validation report 원본 | `ebcb1bf3990d6230d3ec9b8f732eff466345ed620cac7cadf022ff21c4267679` |
| dataset fingerprint | `8e3cacc80b0d6c039ae7fbbd82cec8723967535c5f8e7eaed9b5cd2c7e04bfd3` |
| TRAIN loader fingerprint | `0a2be9910b0ddbe9d523dbc4bcbe2bad52a0acd6974959db620177fdc49f658a` |

dataset manifest hash, 전체 dataset fingerprint, loader fingerprint는 서로 다른 범위의 식별자다. contract의 canonical 의미 hash와 원본 contract JSON file hash 역시 다른 필드이며 임의로 같게 만들지 않았다.

`dataset.json`과 `conversion_attempt_01.json`은 **byte-exact 원본 복사**다. 반면 validation report의 `/dataset_root`, conversion result의 `/route_identity/source_route_path`는 개인 절대 경로 prefix를 **`PORTABLE_E2E_ROOT`로 치환한 공개본**이다. 이 두 파일은 byte-exact라고 주장하지 않는다. 원본 SHA·공개본 SHA와 필드별 redaction map을 provenance에 따로 기록했다. report 안의 기존 원본 hash 참조는 소급 재작성하지 않았다.

validation report의 dataset root가 `.partial`로 끝나는 것은 **최종 rename 전 staging validation의 원본 기록**이다. 최종 데이터셋은 `.partial`이 아닌 위 경로에 실제로 존재하며, publication provenance는 최종 1,980파일을 계수했다. 원본 `created_at_utc`와 simulation timestamp도 보고 시각이나 원격 전송 완료 시각으로 바꾸어 읽지 않는다.

이 폴더는 **compact 증거 묶음이지 전체 데이터셋 다운로드가 아니다**. [dataset.json](dataset.json)이 참조하는 episode·rig·1,968 JPEG 등은 실제 private prepared dataset에 있으며 여기에는 복제하지 않았다. 영상 장면은 중복 없이 [04 경계 PNG 6장·전체 기록 GIF](../04_moving_data_pilot/README.md)를 참고한다.

## 원격 수신·정식 prepared 경로 검증 완료

최종 대상은 **`PERSONAL_DATASET_ROOT/prepared/carla-town07-moving-pilot-20260915-v1`**이다. `PERSONAL_DATASET_ROOT`는 개인 데이터 루트의 공개용 치환이며, 실제 서버 주소·계정·키·개인 절대 경로는 싣지 않는다. `PORTABLE_E2E_ROOT` 역시 각 환경의 저장소 root를 뜻하는 공개용 치환이다.

| 단계 | 실제 확인 |
|---|---|
| 로컬 inventory | 1,980 files / 213,747,472 bytes / 1,968 JPEG / TRAIN 328 |
| 원격 staging 수신 검사 | 로컬 inventory SHA를 외부 기준으로 대조; 1,980개 path·size·SHA 전부 일치, checksum dry-run 변경 0 |
| 새 prepared 경로로 이동 | destination 사전 부재, source/destination 동일 filesystem device 66306, `mv -Tn` exit 0, 이후 staging source 부재 확인 |
| prepared 최종 재검증 | 같은 전체 tree, Common10 planning PASS, TRAIN loader 328 / VAL 0 / TEST 0, 동일 loader fingerprint |
| 실행 범위 | CPU 검증, Torch import 없음, GPU 사용 없음, 설치 0, 원격 native git worktree clean |
| 기존 실행 보존 | duration campaign의 기존 TRAIN 1,147 데이터 변경 없음, 새 데이터 학습 실행 없음, 모델·차량 release 승인 없음 |

원격 검사기 자체의 `dataset_promoted=false`는 **read-only verifier가 이동을 수행하지 않았다는 뜻**이다. 별도의 supervising workflow가 신규 staging dataset만 신규 prepared 경로로 이동했고, 그 뒤 verifier를 다시 실행했다. [handoff.json](handoff.json)이 이 동작을 별도로 기록한다. 기존 데이터셋·실행 중 모델을 덮어쓰거나, verifier의 false를 true로 바꿔 기록하지 않았다.

| 공개 compact 증거 | 대응하는 private 원본 SHA256 |
|---|---|
| [local_inventory_compact.json](local_inventory_compact.json) | `f71ed38454feb1a603075b8d0614223bd56f7d69e2c8373c711b37fe0c02eafb` |
| [staging_verification_compact.json](staging_verification_compact.json) | `2b3786b36252d1252ce55dee8eee9aac9e415f817ecd9517310ca290419cc5e5` |
| [prepared_verification_compact.json](prepared_verification_compact.json) | `4f8d81047a41a6717bbdf631f8ebf99d8a7a23938ebc31c33f17cdcac4eb504c` |
| [handoff.json](handoff.json) | `fa738f16d2d45d5f505cc1d009ccdebd66ad425a881c53c4e850675a290c994f` |

세 inventory/verification의 전체 tree manifest SHA는 모두 `dd56ae009764f9a2ad27407705b598a3a8f6ce02a1f8dc74eb5c78c0b0e64783`으로 일치한다. 공개 검산에서도 private 세 receipt의 1,980개 전체 entry가 같음을 비교하고 로컬 prepared 1,980개 파일을 다시 hash했다. 서로 동일한 5개 검증 dependency의 source SHA와 transfer verifier SHA도 확인했다. 검증기 SHA는 `4a897efdbc21108809b6e15d569b1d7576918fcbe03e3b82704fce993a5f4728`이다.

위 공개본은 **명시적 경로 익명화·compact 사본**이며 byte-exact receipt가 아니다. 큰 `/tree/files` 1,980-entry 배열만 공개본에서 중복하지 않았고, count·bytes·tree hash·validation·loader 결과·범위 제한은 남겼다. private 원본 receipt와 전체 배열은 그대로 보존했다. 각 파일의 `publication_transform`에 생략 필드·개수·redaction map·원본 SHA가 있고, [publication_provenance.json](publication_provenance.json)에 원본/공개본 SHA를 구분했다.

[SHA256SUMS](SHA256SUMS)는 이 새 05 폴더의 다른 모든 파일을 봉인한다. 원본 prepared dataset, 04의 13개 봉인 대상, 상위 `CURRENT`·seal은 이번 공개 작업에서 수정하지 않았다. 수신 검증도 새로운 정지 기능·다중 맵 성능·모델 학습·차량 제어의 증거로 승격하지 않는다.
