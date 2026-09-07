<!-- HH_260906 - Publish compact verifiable transfer evidence without raw image datasets. -->
# Common10 v3 / 개인 학습 서버 전송 검증

`carla-common10-30kph-five-episodes-20260907-v3`는 로컬과 원격 personal venv의 planning 검증을 모두 통과했습니다. 전체 dataset 원본은 Git에 넣지 않습니다. 이 폴더는 검토용 metadata와 무결성 증거입니다.

| Split | Episode | Samples |
|---|---|---:|
| TRAIN | Town07 straight + CTrack left + 새 Town01 right | 1147 |
| VAL | 기존 Town03 right 전체 | 337 |
| TEST | 새 Town04 straight 전체 | 309 |
| 합계 | 5 episodes | 1793 |

기존 세 episode의 split·sample 수·sample JSONL SHA·route geometry SHA는 v2와 동일합니다. Town04 test는 모델·threshold 선택에 쓰지 않습니다.

Dataset fingerprint: `56d9ff663612a090cd61698b53cf0cf39b0a7ae0de6b42d096957a94b6c92047`

Tree manifest: `0ba1d4fbe8544a17a04506b5cb38caf243bee8cbeea4017c9d3a799178d984bb`

파일 10,786개, 디렉터리 43개, 1,033,543,479 bytes가 로컬·원격 staging·원격 prepared에서 일치했습니다. `rsync -rcn --delete`의 변경 목록은 0 byte였습니다. 새 staging만 사용했고 동일 filesystem에서 덮어쓰기 없이 승격한 후 다시 tree/contract를 검증했습니다.

[전송 무결성](01_transfer_integrity.json) · [원격 planning 검증](02_dataset_planning_validation.json) · [기존 split 보존](04_preserved_previous_splits.json)

변환 code와 전체 프로젝트 import closure, model config 등 10개 파일을 reviewed commit `081a71f7fc014d2864b790b0b0cae7378ae18e4c`의 git blob과 byte 단위로 대조해 일치를 확인한 뒤 converter가 지원하는 명시적 commit 옵션을 사용했습니다. 동시에 수정되던 문서·비교기 등은 변환 import 대상이 아니며 [source proof](03_reviewed_conversion_source.json)에 남겼습니다.

로컬 원본: `datasets/prepared/carla-common10-30kph-five-episodes-20260907-v3`

원격 원본: `${PERSONAL_DATASET_ROOT}/prepared/carla-common10-30kph-five-episodes-20260907-v3`

개인 경로는 발행 JSON에서 placeholder로 바꿨습니다. 각 표시본은 원본 report SHA를 포함합니다. 원본 file-by-file manifest는 ignored artifacts의 `transfer/`에 보존했습니다. 발행본은 `sha256sum -c SHA256SUMS`로 확인합니다.

이 전송 작업은 원격 package/Git/GPU를 변경하지 않았습니다. Planning data PASS는 학습 모델 품질이나 closed-loop·실차 승인 PASS를 뜻하지 않습니다.
