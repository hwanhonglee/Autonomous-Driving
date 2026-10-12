# 지도 인식 구조 비교 실제 재학습

<!-- HH_261002 - Separate actual source/CPU preparation from pending structural training and unchanged map acceptance. -->

**두 구조의 고정 20,000회 학습과 전체 350장 평가·독립 재검산을 완료했다.** 17:48:38 KST에 새 v3를 서버 전용 로그로 시작했고, 18시 55분에 소유 프로세스 종료와 두 구조 각각 20,000회 완료를 확인했다. 실제 학습은 대조군 23.01분·후보 38.85분이다. 19시 02분 재검산에서 소스 15개·입력·실행 제공 파일·양쪽 전체 20,000개 기록·최종 숫자 가중치와 저장된 예측의 모든 실제 정답 픽셀을 확인했다. 판정은 **23/27 통과·4개 실패, 미채택**이다. 이 학습은 현재 실행 중이 아니다. 개인 `py312` venv와 GPU0만 사용했고, 뒤의 새 주행 학습은 이 작업 종료 뒤 시작했다. [실제 전체 완료와 독립 재검산](../../../../../../storage/experiments/plans/2026-10-02/temporal_left_right_joint_v2/ROOT_previous_MAPV3_completed.json).

<!-- HH_261002 - Present matched full350 recognition gains and failed recall without claiming an approved HD-map model. -->
| 같은 전체 평가의 지표 | 시작 모델 | 기존 구조 20k | 새 구조 20k |
| --- | ---: | ---: | ---: |
| Town01 표지 IoU | 6.44% | 6.45% | 6.93% |
| Town01 표지 recall | 63.81% | 52.69% | **46.79%로 퇴행** |
| Town01 배경→표지 오탐 픽셀 | 25,548 | 20,791 | 16,702 |
| Town01 차선 IoU | 97.51% | 99.49% | 99.33% |
| Town02 표지 IoU | 75.32% | 78.30% | 79.82% |

표지 오탐과 IoU는 좋아졌지만 Town01 표지 누락이 늘어 시작 모델·대조군 대비 recall의 기존·추가 판정 4개가 실패했다. 기존 차선 보존 기준 통과와 차선 수치가 대조군보다 높다는 주장은 다르다. 같은 TRAIN 지역·이미 알려진 진단 350장의 결과이며 새 독립 장소·실시간 처리·정밀지도 생성 성공은 아니다. 손실 감소나 입력 노출 횟수만으로 지도 모델을 교체하지 않는다.

<!-- HH_261002 - Preserve the pipe-failed V2 and report only actual V3 optimizer journals as running training. -->
이전 v2는 대조군 20,000회 완료 뒤 후보 13,300회에서 **16:04:27 `BrokenPipeError`로 종료**됐다. 후보 최종 가중치와 전체 평가는 없으며 실패 원본은 보존한다. 새 v3는 중간 가중치를 이어 쓰지 않고 두 구조 모두 원래 같은 시작 가중치에서 다시 학습한다. 변경은 실행 식별자와 SSH 출력 대신 서버의 전용 일반 파일로 로그를 보존하는 방식뿐이다. 모델 계산·학습 자료·노출 순서·손실·27개 평가 기준은 그대로다.

## 이번에 바꾸는 부분

지난 두 손실 교정은 표지 오탐을 줄이면서 표지 재현율과 차선 성능을 떨어뜨렸다. 이번에는 손실을 더 추가하지 않고, **세밀한 영상 특징과 넓은 주변 특징을 결합하는 구조**를 바꾼다. 이 설계가 실패 원인을 해결한다는 증거는 아직 없다.

| 비교 조건 | 기존 구조 | 새 구조 |
| --- | ---: | ---: |
| 학습 가능한 파라미터 | 245,831 | 397,175 |
| 추가 학습 | 20,000회 | 20,000회 |
| 입력 노출 | 160,000회 | 160,000회 |
| 학습 자료·순서·crop·정답·원래 CE 손실 | 동일 | 동일 |

둘 다 현재 정상 지도 모델의 같은 가중치에서 시작한다. 새 부분의 최종 출력은 0으로 시작하므로 초기 결과는 기존 구조와 같아야 한다. 학습 중간의 좋은 체크포인트를 골라 쓰지 않는다. 두 최종 모델의 전체 기록과 실제 입력 일치 확인 뒤 같은 350장·기존 27개 기준으로 비교한다.

자료는 기존 TRAIN 1,209개이며, 반복 노출은 새로운 장소나 독립 자료의 증가가 아니다. 기존에 본 350장은 알려진 진단 집합이다. 향상 여부와 별개로 새 장소·날씨 및 실제 실행 속도를 추가 검증해야 한다. 100ms 처리 성능은 아직 측정하지 않았다.

## 완료한 것과 남은 것

완료: 새 v3 소스 전체 검토·독립 검토·각 187개 CPU 합성 검사·서버 소스 15개와 개인 venv의 제공 파일 18개 동일성·실제 TRAIN CPU 사전검사·분리 실행 사전검사. 실행 직전 GPU0의 UUID·사용 프로세스 없음·사용률 0%와 기존 잠금을 확인했다. 실제 두 구조의 CUDA 전방·역전방, 첫 입력의 초기 출력 동일성, loader 0/2 입력 동일성을 확인한 뒤 optimizer 학습을 시작했다. 설치나 GPU1 사용은 하지 않았다.

<!-- HH_261002 - Identify the separate GPU-residue infrastructure revision without changing model mathematics or approval gates. -->
v1의 모델·학습 계산·자료·판정은 그대로 보존했다. v2와 이번에 완료한 v3는 GPU 초기화 전에 물리 GPU0의 정확한 UUID, 다른 GPU 프로세스 0개, 사용률 0%, 드라이버 잔여 메모리 0~16MiB를 두 곳에서 확인했다. GPU1을 쓰거나 다른 작업을 종료하지 않았다. v1의 실제 CPU 검사와 중단된 v2도 이전 근거로 보존하며, 완료한 새 구조 학습은 별도 소스·허가·결과 폴더를 쓰는 v3다.

<!-- HH_261002 - Close the actual fixed10 stored-prediction publication while retaining all four known failures and map capability limits. -->
고정 학습·최종 가중치·전량 동일 입력 기록·350장 전체 평가·27개 수치 독립 재계산은 완료했다. [같은 고정 PNG 10장과 전체 수치](../25_map_native_fusion_v3_results/README.md)도 게시했다. ROOT의 새 발행 코드 41개 CPU 검사와 독립 41개 검사 뒤 실제 원본을 검산해 생성했고, 전체 원본·소스 77개와 10장 전체 픽셀을 다시 확인했다. 실제 Town01·Town02 화면 2장은 직접 열어 확인했다. 저장된 예측을 그린 진단 그림이지 새 추론·실제 Autoware/RViz 촬영·정밀지도 생성 성공이 아니다. 기존 writer의 metadata SHA가 요약에만 추가되는 자료 형식 차이는 새 발행 v2에서 정확히 분리했으며 동결된 v1과 학습 결과·모델·판정은 변경하지 않았다. [생성 뒤 전체 재검증](../../../../../../storage/experiments/diagnostics/2026-10-02/map_native_fusion_v3_gallery_postcheck_v2/ROOT_actual_whole350_and_png10_postcheck.json), [읽기 전용 재현 코드](../../../../../../storage/experiments/diagnostics/2026-10-02/map_native_fusion_v3_gallery_postcheck_v2/reproduce_read_only_postcheck.py)를 보존한다.

남은 것은 표지 누락이 남는 조건 분석, 실행 속도 계측·새 환경 검증이다. 더 많은 optimizer 업데이트 자체를 해결책으로 단정하지 않는다. 현재 GPU0에서는 별도 [새 좌우회전 주행 모델](../26_actual_left_right_joint_learning/README.md)을 학습 중이며 지도 후속 GPU 작업은 겹치지 않는다.

현재는 RGB 7종 인식 연구 모델이다. 정밀지도 생성 완성품이나 신호등 상태 판독, 메인 9개 기능 승인으로 세지 않는다. 기존 지도 모델을 자동 교체하지 않는다.

- [사전 고정 구조·동일 노출 조건](../../../../../../storage/experiments/plans/2026-10-02/map_native_fusion_pair_v1/ROOT_prospective_protocol_and_equal_schedule.json)
- [v2 ROOT 학습 범위·대기 조건](../../../../../../storage/experiments/plans/2026-10-02/map_native_fusion_pair_v2/ROOT_fixed20k_structure_pair_authorization.json)
- [v3 별도 고정 학습 범위](../../../../../../storage/experiments/plans/2026-10-02/map_native_fusion_pair_v3/ROOT_fixed20k_structure_pair_authorization.json)
- [v3 서버 로그 분리 실행 범위](../../../../../../storage/experiments/plans/2026-10-02/map_native_fusion_pair_v3/ROOT_detached_launch_authorization.json)
- [v3 실제 TRAIN CPU 검사](../../../../../../storage/experiments/training/2026-10-02/map_native_fusion_dispatch_v3/ROOT_actual_TRAIN_CPU_preflight.json)
- [v3 전체 완료 결과·양쪽 최종 기록](../../../../../../storage/experiments/training/2026-10-02/map_native_fusion_pair_v3/summary.json)
- [v3 전체 저장 예측과 분류 수치](../../../../../../storage/experiments/training/2026-10-02/map_native_fusion_pair_v3/known_evaluation.json)
- [v3 실제 52개 파일의 로컬 전량 일치](../../../../../../storage/experiments/training/2026-10-02/map_native_fusion_dispatch_v3/ROOT_all52_local_copy_verified.json)
- [v3 실제 분리 실행 영수증 — 자체로 학습 증거는 아님](../../../../../../storage/experiments/training/2026-10-02/map_native_fusion_dispatch_v3/ROOT_actual_detached_launch_receipt.json)
- [v2 실제 서버 TRAIN CPU 검사](../../../../../../storage/experiments/training/2026-10-02/map_native_fusion_dispatch_v2/ROOT_actual_TRAIN_CPU_preflight.json)
- [v2 실제 GPU 학습 시작·13,300회 확인](../../../../../../storage/experiments/training/2026-10-02/map_native_fusion_dispatch_v2/ROOT_actual_GPU_fit_progress_1531.json)
- [이전 v1 실제 CPU 검사 — GPU 학습 미실행](../../../../../../storage/experiments/training/2026-10-02/map_native_fusion_dispatch_v1/ROOT_actual_TRAIN_CPU_preflight.json)
- [이전 손실 비교의 전체 결과·실제 화면](../19_map_sign_other_pair_loss/README.md)
- [완료된 주행 학습·이어지는 실제 좌회전 비교](../20_actual_low_speed_left_joint_learning/README.md)
