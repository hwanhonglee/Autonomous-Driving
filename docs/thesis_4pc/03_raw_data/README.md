# 논문 원본 주행과 시험 데이터

<!-- HH_261007 - Retain raw data, failed attempts and exact acquisition roots while exposing all PC owners in one catalog. -->

<!-- HH_261007 - Point current navigation to the observed home archive without changing frozen acquisition identifiers. -->

| 원본 구분 | 위치 | 보존 이유 |
| --- | --- | --- |
| 2025 실제 주행 | [실차 bag](/home/a/thesis_4pc/VILS_Test/2025_VILS_Test_bag_real) | 기존 시나리오·분석의 원본 |
| 2025 가상 주행 | [가상 bag](/home/a/thesis_4pc/VILS_Test/2025_VILS_Test_bag_virtual) | 일부가 과거 PC4 실행의 고정 입력으로 사용됨 |
| 2026 PC1 게시본 | [PC1 결과](../../vils/result/PC1/20260814T091657Z-VILS_PC123_REAL_20260814_1816_R01/README.md) | 복구한 차량·계획·통신 기록과 분석 |
| 2026 PC2 게시본 | [PC2 결과](../../vils/result/PC2/20260814T091155Z-pc2-power-interrupted/README.md) | 전원 중단과 수집 누락 자체가 시험 근거 |
| 2026 PC3 게시본 | [PC3 결과](../../vils/result/PC3/20260814T091657Z-VILS_PC123_REAL_20260814_1816_R01/README.md) | 위치 추정·cross-PC 기록·복구 근거 |
| 2026 PC4 게시본 | [PC4 결과](../../vils/result/PC4/20260814T092321.301564497Z-outdoor_route_aligned_shadow_response-b0fed535-04fb-41f6-85e7-48e04c426867/README.md) | 실제 부분 시험의 전달·경로·Planning 반응 |
| PC4 개발 시험 전체 | [원본 run 폴더](/home/a/carla-autoware-universe/autoware/pc4_vils_runs) | 사전 시험·실패·중단·최종 결과 모두 포함 |
| 이전 문서 저장소 | [기존 VILS clone](/home/a/Autonomous-Driving-vils-docs) | 현재 결과와 동일한 전체 사본인지 아직 전량 검증하지 않음 |

2025 자료의 현재 위치는 `/home/a/thesis_4pc/VILS_Test`입니다. 앞선 목록의279개 파일은 모두 현재 위치에 있고 파일 세대 정보도 일치합니다. 앞선 목록에는 내용 해시가 없으므로 이 비교를 전체 바이트의 동일성 검증으로 부르지는 않습니다. 과거 PC4 스크립트에는 이전 절대경로가 남아 있어 그대로 실행하지 마세요. [재현 전 필요한 확인](../05_reproduction/README.md)을 따릅니다.

PC4 사전 주행 전환 자료 약1.39GB도 실제 bag이며 임시 쓰레기로 분류하지 않습니다. 게시본과 읽기 전용 원본은 경로·권한·일부 YAML 바이트가 달라, 크기만 보고 중복 삭제하지 않습니다.

이 폴더는 탐색 안내이며 원본을 옮기거나 다시 기록하지 않습니다. rosbag을 차량 연결 ROS graph나 CAN 송신 환경에서 재생하지 않습니다.

[전체 분류로 돌아가기](../README.md)
