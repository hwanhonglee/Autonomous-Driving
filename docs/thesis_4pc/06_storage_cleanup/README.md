# PC 디스크 용량과 정리 후보

<!-- HH_261007 - Separate protected toolchains and irreplaceable research from candidates needing inactive-use and backup checks. -->

<!-- HH_261007 - Keep the original capacity snapshot distinct from the later real home relocation. -->

**최신 자료 위치는 홈의 [thesis_4pc](/home/a/thesis_4pc/README.md)입니다.** Documents·Downloads의154개 파일 약2.59GB를 실제 분류 이동했고 영구 삭제는 하지 않았습니다. 아래는 같은 날15시 전후의 최초 용량 조사입니다. 이동이 여유 공간 확보를 뜻하지 않으며 후보의 크기를 삭제 가능량으로 읽지 마세요.

2026년 10월 7일 15시 전후, 사용자에게 쓰기 가능한 공간은 약4.47GB였습니다. 아래 값은 GB를 10억 byte로 표시한 할당 용량이며 포함 관계가 있으므로 합산하지 않습니다.

<!-- HH_261007 - Preserve the historical capacity table while exposing the current separately observed VILS archive path. -->

아래 표의 `/home/a/VILS_Test`는 최초 조사 당시 위치입니다. 현재 해당 자료는 [홈의 VILS_Test](/home/a/thesis_4pc/VILS_Test)에 있으며, 옛 목록·용량 영수증은 당시 기록 그대로 유지합니다. 경로 변경을 디스크 여유 공간 확보로 세지 않습니다.

## 유지할 기본 환경과 원본

| 위치 | 할당 용량 | 처리 |
| --- | ---: | --- |
| `/home/a/UnrealEngine_4.26` | 130.85GB | 보존 |
| `/home/a/carla` | 122.85GB | CARLA 소스·개발 환경 보존 |
| `/home/a/carla-autoware-universe` | 98.25GB | CARLA 패키지·Autoware·맵·PC4 환경 보존 |
| 현재 저장소 `storage/experiments` | 132.67GB | 학습·주행·실패 원본 보존. 용량이 커도 자동 삭제 금지 |
| 현재 저장소 PC1–4 게시본 | 1.22GB | 논문 근거 보존 |
| 외부 `pc4_vils_runs` | 2.01GB | 개발·사전 시험·최종 부분 run 모두 보존 |
| `/home/a/VILS_Test` | 0.55GB | 2025 논문 자료와 고정 실행 입력 보존 |
| RoadRunner·관련 프로젝트 | 설치 위치와 프로젝트 별도 확인 | 보존 |

## 삭제가 아니라 확인부터 필요한 후보

| 후보 | 확인한 용량 | 삭제 전 필요한 조건 |
| --- | ---: | --- |
| `/var/tmp/ccache`의 본인 M/R 데이터 | 약1.80GB | 기존 승인 유지. 새 비사용 검사와 개인 서버 백업·전량 검증 후 제거. 통계·TAG·폴더 유지 |
| 현재 저장소 `autonomy/log` | 약4.39GB | run별 내용과 기존 증거 참조·현재 열린 파일 확인. 논문·실패·진단 로그는 먼저 보존 |
| `/home/a/.ros/log` | 약20.60GB | 오래된 ROS 진단 로그78,159개. 조사된 최신 수정 시점은9월24일. 논문·실패 근거 보존과 비사용 확인 전 제거 금지 |
| `snap/code/263`·`267`의 휴지통 | 일반파일 약16.06GB | 고유inode239,912개. 모든 하드링크 별칭이 같은 조사 범위에 있어 외부 공유는 관측되지 않음. 복구 필요성과 최종 비사용 확인 후 별도 승인 필요 |
| 현재 저장소 `autonomy/build` | 약1.97GB | 설치·실행·재빌드 의존과 기존 정리 이력 확인. 전체 삭제는 Autoware 보존 요구와 별도 판단 |
| 이전 `Autonomous-Driving-vils-docs` clone | 약1.41GB | 약0.705GB의 Git LFS 원본·이력이 포함돼 보존. 전체 대체 사본 확인 없이 clone/LFS 제거 금지 |
| Downloads의 지도·VILS ZIP4개 | 약1.03GB | 추출본·원본·백업의 전량 동일성을 아직 확인하지 않아 보존 |
| `/tmp`의 이전 검사 폴더 | 조사 시 약0.57GB | 소유권·열린 파일·수정 소스·백업 여부를 폴더별 확인. 일괄 삭제 금지 |

위 용량은 **삭제 가능 용량이 아닙니다.** 사용 중 여부와 내용 보존을 아직 충족하지 않은 후보는 그대로 둡니다. 이름에 `tmp`, `failed`, `copy_org`, `old`가 있어도 실험 근거나 고유 변경이 있으면 보존합니다.

<!-- HH_261007 - Preserve the directly referenced failure log and separate hardlink recovery bounds from deletion authorization. -->

ROS 로그 상위10개 약11.53GB는2025년4–5월 파일입니다. 다만 [PC1 작업 이력](../../vils/VILS_PC1_WORK_HISTORY.md)의228행 부근이2026년8월10일의 `launch.log`를 실제 실패 근거로 참조하므로 폴더 전체를 제거하지 않습니다. 우선 필요한 기록을 제외하고 압축 보존할 범위를 결정합니다.

휴지통 두 곳의 일반파일 할당량은 총16,058,953,728byte입니다. 링크 관계상 회수 가능한 양이지 비사용·내용 보존·영구 삭제 승인은 아닙니다. 링크1,256개는 따라가지 않았으며 이 중6개가 시스템 Python을 가리켜, 이후 정리에서도 링크 대상인 `/usr/bin/python3`는 유지해야 합니다. `/home/a/.local/share/Trash`는 이번 조사에서 빈 폴더뿐이었습니다.

15시 전후의 최초 목록 생성 단계는 원본 이동0개·삭제0개였습니다. 이후16시31분에는 Documents·Downloads만 실제 이동했고, 설치 환경·고정 시험 원본·휴지통·ROS 로그·캐시는 제거하지 않았습니다. 서버 백업 경로는 10월7일 읽기 전용 접속에서도 `No route to host`였으므로, 승인된 캐시도 백업 없이 삭제하지 않았습니다. [앞선 승인·백업 실패·전량 보존 기록](../../../storage/experiments/maintenance/2026-10-06/ccache_mr_v1/ROOT_failed_backup_whole_local_postcheck_v1.json)을 참조합니다.

[실제 파일 목록과 용량 기록](../../../storage/experiments/maintenance/2026-10-07/thesis_4pc_catalog_v1/catalog.json), [전체 분류로 돌아가기](../README.md)
