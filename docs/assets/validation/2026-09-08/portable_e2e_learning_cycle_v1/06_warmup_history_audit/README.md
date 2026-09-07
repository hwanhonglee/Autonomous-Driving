# 06 — Warmup 학습 앵커와 과거 입력 감사

<!-- HH_260906 - Correct historical anchor accounting and leave any future history-only policy explicitly unadopted. -->

기존 데이터에는 **정지 warmup도 학습·validation 앵커로 포함**되어 있습니다.
9월 5일 문서의 “warmup과 tail을 모두 학습 앵커에서 제외했다”는 문장은 사실과 달라
바로잡았습니다. 당시 v2의 warmup은 train **70/613개**, val **35/337개**입니다.
데이터·기존 평가 수치·337개 validation 분모는 바꾸지 않았습니다.

## 실제 포함 개수

아래는 v3의 train/val sample ID를 원래 CARLA 프레임 번호와 `capture_phase`에
대응시킨 결과입니다. 앵커는 현재 입력과 미래 정답을 묶는 한 학습·평가 시점입니다.

| Episode / split | 포함된 전체 앵커 | warmup 앵커 | driving 앵커 | 원본에만 있는 tail 앵커 |
|---|---:|---:|---:|---:|
| Town07 직진 / train | 309 | 35 | 274 | 65 |
| C-track 좌회전 / train | 304 | 35 | 269 | 65 |
| Town01 우회전 / train | 534 | 35 | 499 | 65 |
| **v3 train 합계** | **1,147** | **105** | **1,042** | **195** |
| Town03 우회전 / val | **337** | **35** | **302** | **65** |

네 episode 모두 warmup은 3.5초(20 Hz physics 70 tick), 카메라 앵커는 10 Hz 35개입니다.
`stationary_tail` 6.5초·65개는 미래 정답을 완성하는 원본 문맥으로 남지만
학습·평가 앵커에서는 제외됩니다. v2의 세 sample JSONL은 v3의 대응 파일과
**바이트가 동일**함을 확인했습니다. Town01 추가분을 과거 v2 분모에 섞지 않았습니다.

## Warmup은 학습 대상이면서 과거 입력이기도 함

현재 ego history는 **현재 1개 + 과거 9개**, 합계 10개 상태를 사용합니다.
각 episode의 warmup 35개 중 26개는 이미 10칸 모두 유효한 과거 입력 mask를 가집니다.
첫 driving 앵커의 sequence index는 35이며, index 35–43의 첫 9개 주행 앵커는
warmup 상태를 각각 9, 8, …, 1칸씩 과거 입력으로 사용합니다.

수집기는 warmup에서 완전 제동을 3.5초 유지한 뒤 정해진 순서에 따라 BasicAgent로
넘깁니다. 현재 입력의 13개 상태·명령 feature와 이미지·보정·경로·history/mask에는
**명시적인 출발 countdown이나 engagement/release flag가 없습니다.**
시각 정보, 상태 변화, mask, 경로 등에 간접 단서가 있을 수 있으므로 “모든 입력이 같아
출발을 전혀 관측할 수 없다”거나 “이것이 모델 오차의 원인이다”라고 증명한 것은 아닙니다.

## 향후 history-only 정책은 아직 채택하지 않음

warmup을 향후 학습 대상에서만 빼고 과거 문맥으로 보존하는 방안은 **제안일 뿐,
정책 채택·구현·기존 데이터 필터링은 하지 않았습니다.** 현재 loader는
`self.examples` 안에서만 과거 상태를 찾습니다. 여기서 warmup 행을 단순 삭제하면
첫 9개 driving 앵커의 과거 입력까지 잃으므로 다음 계약이 먼저 필요합니다.

- 전체 causal history 저장소와 최적화·평가 앵커 인덱스를 분리합니다.
- 출발의 외부 제어권 경계를 명시하고, 새 입력이 필요하다면 별도 ABI로 사전 선언합니다.
- 새 데이터 버전·train/val 정의·분모를 학습 전에 고정합니다. 기존 val337과 같은 평가라고 부르지 않습니다.
- driving 중 신호 대기·정상 정지 라벨은 보존합니다. 모든 0속도 표본을 일괄 제거하는 정책이 아닙니다.

v3에서 warmup 수만 뺀 산술값은 train1,042/val302이지만, 이는 **실제로 생성하거나
평가한 새 데이터셋의 수치가 아닙니다.** 기존 split·label·mask·모델·안전 기준은 유지합니다.

## 검증 범위와 출처

[집계·입력 계약 JSON](warmup_history_audit.json), [원본 출처 30개 SHA-256](provenance.json),
[공개 파일 검증 목록](SHA256SUMS)을 함께 보관합니다. 30개 원본 해시는 공개 직전 다시
일치함을 확인했고, 추가로 v2/v3 세 sample 파일의 동일성도 대조했습니다. 원본 상대 이름은
private 데이터 식별자이지 다운로드 링크가 아닙니다. 원본 검토 보고서의 SHA와 이 공개
요약본의 SHA를 구별하며, 별도 커밋된 감사 실행기가 있다고 주장하지 않습니다.

train/val의 sample JSONL·원본 프레임 메타데이터·수집 계약·loader 소스를 읽었습니다.
test는 episode 메타데이터로 split만 확인했고 **test sample·이미지·정답 payload는 읽지
않았습니다.** 카메라 픽셀, 모델 예측·성능 결과, 시뮬레이터·원격 GPU는 사용하지 않았습니다.
이 자료는 데이터 계약 감사이며 주행 성능이나 실차 제어 승인이 아닙니다.

[정정한 9월 5일 보고서](../../../../../validation-2026-09-05-portable-e2e-common10-30kph.md) ·
[최신 결과 묶음](../README.md)
