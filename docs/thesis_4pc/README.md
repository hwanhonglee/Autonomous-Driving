# 졸업논문 PC 4대 시험 자료

<!-- HH_261007 - Group thesis navigation without relocating frozen data, installed tools or active runtime inputs. -->

<!-- HH_261007 - Point to the actual home consolidation while keeping frozen four-PC source inputs and earlier audit receipts intact. -->

**자료를 찾는 시작 위치는 홈의 [thesis_4pc 안내](/home/a/thesis_4pc/README.md)입니다.** 16시31분에 완료한 Documents·Downloads 분류 이동은 파일·폴더26개, 내부 일반파일154개 약2.59GB이며 전체 해시를 확인했습니다. 이 저장소의 안내는 PC1부터 PC4까지 기존 VILS 개발 이력과 시험 원본을 연결합니다. 기존 PC1–4 게시본·전체 PC4 개발 시험·문서 저장소·Unreal 프로젝트는 원래 위치에서 찾습니다.

<!-- HH_261007 - Report observed current VILS locations without rewriting the earlier immutable move plan or inferring who moved them. -->

2025 실차·가상 시험과 분석은 현재 [홈의 VILS_Test](/home/a/thesis_4pc/VILS_Test)에 있습니다. 이전 `/home/a/VILS_Test`는 현재 존재하지 않습니다. 이 위치 확인은 앞선 Documents·Downloads154개 이동 기록에 포함되지 않으며, 과거 실행 스크립트의 고정 입력 경로를 자동으로 수정한 것도 아닙니다. [원본 위치와 재현 제한](03_raw_data/README.md)을 확인하세요.

이 자료는 차량과 가상환경을 연결하는 VILS 논문 자료입니다. 현재 개발 중인 portable E2E 모델의 기능 승인 자료와는 별개입니다. 중단·실패 시험도 논문 근거로 함께 보존합니다.

## 원하는 자료 찾기

| 분류 | 들어 있는 내용 |
| --- | --- |
| [01 시스템 구조와 PC별 변경 이력](01_system_and_history/README.md) | 각 PC의 역할, 공통 구조, 개선·구현·제한 이력 |
| [02 시험 준비와 실행 조건](02_test_preparation/README.md) | PC별 준비 사항, topic 계약, 단계별 시험과 복구 절차 |
| [03 원본 주행과 시험 데이터](03_raw_data/README.md) | 2025 실차·가상 bag, 2026 PC1–4 게시본, PC4 전체 개발 시험 |
| [04 논문 분석과 그림 표](04_paper_analysis/README.md) | 통신·경로·시스템 분석, 재현 스크립트, 주장 가능 범위 |
| [05 재현과 원본 보존](05_reproduction/README.md) | 검사 방법, 실행 경로 유지 이유, 안전한 분석 원칙 |
| [06 디스크 용량과 정리 후보](06_storage_cleanup/README.md) | 보존할 기본 환경, 용량 조사, 삭제 전 필요한 확인 |

기존4-PC 원본 위치는 [앞선 기계 판독 목록](../../storage/experiments/maintenance/2026-10-07/thesis_4pc_catalog_v1/file_locations.csv), 용량과 검사 범위는 [앞선 원본 목록 기록](../../storage/experiments/maintenance/2026-10-07/thesis_4pc_catalog_v1/catalog.json)에 있습니다. 이 앞선 목록은 파일명·크기·세대를 조사한 결과이며 전체 내용의 SHA 검증이나 비사용 증명은 아닙니다. 여러 분류에 같은 원본이 포함돼 있으므로 분류별 용량을 더하면 중복됩니다. Documents·Downloads의 실제 이동에는 별도의 [이동 전후 경로표와 전체 해시·원복 방법](/home/a/thesis_4pc/90_move_records_and_cleanup/README.md)을 사용합니다.

## PC 4대 실제 시험을 읽는 방법

| PC | 역할 | 게시된 실제 결과의 범위 |
| --- | --- | --- |
| PC1 | 계획·차량 제어·CAN | 전원 중단 후 복구한 기록. 통신·시스템 분석에 사용 가능하지만 PC4 때문에 제동했다는 인과 증거는 없음 |
| PC2 | 실제 인지와 가상 객체 검증·융합 | 전원 중단 부분 자료. PC2 rosbag은 수집되지 않았음 |
| PC3 | 위치 추정·지도·TF | core 기록과 전원 중단 cross-PC 기록. 복구 이력과 한계를 함께 사용 |
| PC4 | CARLA 가상 객체와 연동 | 636초 부분 시험. PC4 내부 Planning 반응과 전달 기록은 있으나 예정 660초 nominal 시험은 미완료 |

PC1과 PC3의 연계 근거는 해당 결과에 설명돼 있습니다. PC2·PC4의 다른 run 이름을 같은 시각이라는 이유만으로 하나의 성공 run으로 합치지 않습니다. 네 PC 자료가 존재한다는 사실도 완전한 4-PC 안전 시험 통과를 뜻하지 않습니다.

2026년 10월 7일 15시 전후 용량 조사에서 공개 PC1–4 자료는 할당 약1.22GB, 별도 PC4 원본 전체는 약2.01GB, 2025 자료는 약0.55GB였습니다. 정확한 바이트와 원래 위치는 목록 기록을 사용하세요.
