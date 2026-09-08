# 실제 학습 비용 계측 — 같은 16-step 학습 두 번

<!-- HH_260906 - Publish real operator observations separately from synthetic checks, full campaigns, throughput claims and deployment approval. -->

기존 학습 코드를 그대로 사용해 **짧은 실제 학습 2회**를 실행했습니다. A는 계측 없이,
B는 CPU/CUDA profiler를 켜고 각각 16 optimizer step·64 sample 노출을 진행했습니다.
1540-step 전체 학습 캠페인 2회를 추가한 것이 아니며, 합성 연산 벤치마크도 아닙니다.

2026-09-09 03:44:20–03:45:28 KST에 기존 개인 venv와 지정 GPU0에서 수행했습니다.
원래 physical-v1 모델 954,590 parameters, 전체 v3 개발용 train 1,147개/3 episode,
seed 20260903, batch 4, LR 0.0001, `num_workers=0`, 기존 손실·검사 항목을 유지했습니다.
초기·최종 full-corpus 무결성 검사는 test JSON/JPEG를 읽을 수 있지만,
test는 신경망 입력·손실·평가·선택에 사용하지 않았습니다.

## 실제 관측된 비용

![실제 16-step 학습에서 CPU에 귀속된 self time](01_measured_cpu_self_cost.png)

| B의 16-step 계측 범위 | 관측값 |
|---|---:|
| Single-process DataLoader CPU self time | 805.944 ms |
| DataLoader inclusive CPU time | 953.460 ms, 배치당 약 59.591 ms |
| Host `cudaLaunchKernel` CPU self time | 582.549 ms / 182,333 호출 |
| profiler가 보존한 CUDA kernel events | 190,041개 |
| 전체 trace events | 1,469,522개 |

배치 준비와 많은 작은 커널의 CPU 호출 비용이 다음 최적화 후보로 관측됐습니다.
DataLoader 시간에는 이미지 읽기·해시·변환·텐서 준비 등이 포함됩니다.
**JPEG 디코딩만의 비용을 분리해 측정한 것은 아닙니다.** stack을 기록하지 않아
커널 비용을 특정 소스 줄 하나의 원인으로 단정하지 않습니다.

위 CPU self time은 연산에 귀속된 시간입니다. 전체 wall 시간이나 GPU 사용률이 아닙니다.
inclusive 연산은 중첩되고, CPU 연산의 device time과 같은 CUDA event도 중복될 수 있어
이들을 더해 GPU busy 비율로 표시하지 않습니다. 안전 검사도 그대로 실행했으며 제거하지 않았습니다.

[전체 원본 연산 집계](raw/key_averages.json) · [해석과 한계](raw/profile_analysis.json).

## 결과가 같았는지

- A/B의 16줄씩, **총 32개 학습 기록은 파일 바이트까지 동일**했습니다. 초기 step도 제외하지 않았습니다.
- 종료 시점의 가중치·optimizer·CPU/CUDA RNG·설정·장치 ABI는 실행기가 weights-only 로더로 비교해
  정확히 일치했습니다. 유일한 제외 항목은 checkpoint 최상위 `created_at_utc`입니다.
- 완료 자료 수집기는 checkpoint를 다시 역직렬화하지 않았으며, 서버에서 두 파일의 SHA만 확인했습니다.
- 첫 loss 8.856083, 마지막 loss 6.642898은 작은 진단 실행의 학습 값이지, 성능 향상이나 수렴 증명이 아닙니다.

[A의 16개 원본 기록](raw/A_unprofiled/metrics.jsonl) · [B의 16개 원본 기록](raw/B_cpu_cuda_profiled/metrics.jsonl) ·
[원본 완료 보고서](raw/report.json) · [전송·해시 검증](raw/transport_verification.json).

## 계측 자체의 시간은 별도

| 구간 | 시간 |
|---|---:|
| A train 함수 + 마지막 GPU 동기화 | 3.380 s |
| B train 함수 + 마지막 GPU 동기화 | 2.763 s |
| B profiler context 시작·종료 포함 | 6.686 s |
| B context 종료 이후 보고 완료까지 | 52.319 s |
| 위 후처리 중 별도로 측정한 최종 corpus 검사 | 4.733 s |

**B가 빨라졌다는 결론은 낼 수 없습니다.** A→B 고정 순서, 초기화·캐시와 profiler 부담이 다릅니다.
B 후처리의 나머지 약 47.586초에는 집계·trace 저장/파싱·checkpoint 비교·소스 확인 등이
분리되지 않은 채 포함됩니다. 따라서 “trace 저장만 47.586초/65초 걸렸다”라고 말하지 않습니다.
전체 보고 구간은 약 68.156초였습니다.

## 원본 보존과 제외 범위

trace는 **528,981,135 bytes(약 504.5 MiB)**로 승인된 200 MiB 전송 상한을 넘어 서버에만 보존했습니다.
checkpoint도 각각 11,519,247 bytes인 두 파일을 전송하거나 게시하지 않았습니다.
서버의 원본 25개 파일을 전후 해시 확인했고, 원래 manifest의 24개 payload 검증이 통과했습니다.
로컬에는 그중 22개를 정확하게 복사했습니다. 게시물에는 필요한 텍스트만 다시 선택했습니다.
서버에 남은 세 파일의 이름·크기·SHA는 [검증 기록](raw/transport_verification.json)에 남아 있습니다.
원래 원격 manifest는 [참조용 원문](remote_manifest_original.sha256)이며,
게시 폴더의 검증에는 이 폴더의 [SHA256SUMS](SHA256SUMS)를 사용하세요.

학습 소스는 기존 `b478f02e…`의 11개 파일과 일치하며, 별도 profiler 실행 소스는 `a995722f…`입니다.
둘을 같은 Git 실행 버전이라고 섞어 쓰지 않습니다.
[사전 계획](raw/plan.json) · [실행한 profiler 원본](source/executed_profiler.py) · [게시 provenance](provenance.json).

03:48:33 KST 확인 당시 GPU0은 0%·0 MiB, compute process 없음, 작업 잠금도 해제된 상태였습니다.
이는 **종료 후 관측값**이며 실행 중 사용률 측정이 아닙니다.
[종료 후 확인](raw/completion_observation.json).

기존 corpus의 warmup·물리적으로 불일치하는 일부 target 한계는 그대로 남아 있습니다.
새 실패 수집본을 승인하거나 데이터·물리 한계를 바꾸지 않았습니다. 모델 배포·차량 제어 승인은 없습니다.
다음 실행은 배치 준비나 커널 호출을 별도로 좁혀, 입력·학습 결과의 동등성을 유지하는지 먼저 검증해야 합니다.
