# 현재 Autoware 소스 게시와 다음 모델 학습

<!-- HH_261012 - Separate annotated source reproduction and preparatory lifecycle checks from training and vehicle approval. -->

<!-- HH_261012 - Record successful publication separately from the still-blocked training and unfinished vehicle integration. -->

**18:37 KST 게시 완료:** 상세 공통 커밋 `691c38c`, 플랫폼 `522f94a`, 스캐폴드 `2eb9583`을 GitHub에 atomic push했고 원격 해시 일치까지 확인했습니다. 이전 미게시 커밋 3개도 포함되며 force push는 하지 않았습니다. 플랫폼·스캐폴드는 공통 기준선에서 목적 문서와 README만 변경한 준비 브랜치입니다. **18:39 재조회에서도 학습은 미시작:** 로드된 NVIDIA 580.173.02, 설치 모듈과 NVML 580.178.04의 불일치로 물리 GPU 0 조회가 여전히 종료값 18에 실패했습니다. [게시 성공·경고·첫 검사 오류와 후속 확인](publication_verified_v1.json)을 보존합니다. 아래는 구현·검사의 상세 범위입니다.

현재 프로젝트의 `autonomy/src`와 `autonomy/install`을 기준으로 영어 주석과 소스 재현 패치를 정리했습니다. 별도 PC4/VILS Autoware는 변경하지 않았습니다. 정확한 원본 커밋과 4개 composite patch를 보존했고, 격리된 로컬 체크아웃에서 수정 파일 **68/68개**의 바이트와 Git 실행비트를 재현했습니다. 이는 소스 복원 검증이며 전체 빌드나 실차 주행 검증이 아닙니다.

## Autoware 주석과 재현 결과

의존 저장소 32개 중 실제 변경은 4개입니다. core 1.9.0, universe·launch 0.52.0, lanelet2 extension 1.2.0이며 각 exact HEAD는 [snapshot manifest](../../../../../autonomy/source_snapshots/2026-10-12/active_v1/manifest.json)에 있습니다. tracked 변경 53개와 새 소스 15개를 함께 보존했습니다. ignored 소스 폴더를 root Git에 통째로 추가하거나 upstream 저장소에 push하지 않습니다.

| 대상 | 보완 내용 | 검증 |
| --- | --- | --- |
| 의존 소스 변경 68개 | 기존 HH 주석 6개 유지·빠진 62개 추가 | 새 주석을 제외하면 원래 전체 바이트·파일 모드 일치 |
| 자체 launch·호환 소스 126개 | 기존 55개 유지·48개 직접 추가·23개 별도 설명 | 직접 주석의 원래 바이트 복원·봉인 설정의 SHA 유지 |
| 환경 진입점 | canonical ROS 환경 선택 설명 추가 | `bash -n` 문법 검증 |
| source snapshot | 32개 exact HEAD·4개 patch·15개 새 소스·SHA | patch 복원 4/4 성공·68/68개 일치 |

실제 복원은 **3.59초**, 임시 할당 약 **192.7MB**였습니다. 4개 sparse checkout의 변경 대상만 검증했으며 전체 clean 소스, 네트워크 clone, 컴파일 결과 또는 설치 바이너리를 전량 인증한 것은 아닙니다. [독립 복원 검산](source_replay_independent.json), [의존 소스 주석 검산](../../../../../autonomy/provenance/2026-10-12/nested-source-comments-v1.json), [자체 코드와 봉인 설정 설명](../../../../../autonomy/annotations/2026-10-12/autoware_comments.json)을 보존합니다.

## 로컬 검사와 실행 환경

현재 Autoware overlay를 명시적으로 source한 검사에서 launch·메시지 계약·좌표·shadow 연결 회귀 **373개**가 통과했습니다. 실제 CARLA·ROS 노드·차량을 구동한 시험은 아닙니다.

최종 반복 검사도 **373개 통과·13.89초**였습니다. 앞선 정상 환경 검사의 13.90초와 별도 실행이며 실제 차량 주행 성능 인증으로 합산하지 않습니다.

앞선 두 검사 시도도 보존합니다. 첫 직접 pytest 호출은 `portable_e2e` 검색 경로가 없어 수집 오류 1건이었고, 다음 workspace Python 호출은 외부 PC4 ROS 경로가 섞여 메시지 import 수집 오류 2건이었습니다. canonical 환경을 source한 후의 결과와 구분하며 검사 실패를 실차 기능의 실패로 설명하지 않습니다.

```bash
source tools/e2e/env.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 CUDA_VISIBLE_DEVICES='' \
  python3 -B tools/workspace_python.py -m pytest -- \
  -q -p no:cacheprovider autonomy/autoware_e2e_vad_launch/test
```

새 snapshot 도구의 임시 Git fixture 검사 **33개**는 ROOT 재검사에서 **3.17초**에 통과했습니다. 원래 저장소 상태 보존, tracked·새 파일 패치, 비밀값·symlink·기존 출력·변경 중 입력의 거부와 저장 체크섬을 검사했습니다. 현재 레거시 root 링크·workflow 바로가기는 **0개**, 경로 오류도 **0개**입니다.

게시 대상의 일반 파일 6,471개와 gzip 56개 전체를 알려진 개인키·PAT·AWS 패턴으로 검사했습니다. 실제 비밀값으로 판정된 항목은 없었으며, 의도적인 거부 테스트와 계약 digest 치환표식을 구분했습니다. 압축 파일은 총 2,455,005,595바이트를 EOF까지 읽었고, 초기 제한으로 미완주했던 범위와 이후 완료 검사를 함께 보존합니다. 이는 모든 형태의 비밀값 부재를 보장하는 인증이 아닙니다. [게시 보안 검사와 전량 범위](publication_security_review.json)를 확인할 수 있습니다.

전체 staged 공백 검사에는 과거 실험·봉인 소스의 trailing whitespace와 EOF 공백 경고가 남아 있습니다. 원래 봉인된 바이트를 보존하기 위해 이를 자동 포맷하거나 제거하지 않습니다. 새로 작성한 파일의 검사와 과거 경고를 구분합니다.

별도 검산에서는 전체 공백 경고 1,980개 중 1,857개가 기존 봉인 CSV 한 파일에 있었고, 새 10월 12일 작업의 선택 78파일은 공백 경고 0개였습니다. 안내 8문서의 working/staged 링크 124회 확인에서도 누락 0개였습니다.

최종 Git 게시 검사는 첫 시도 중 ROOT의 새 문서 stage가 겹쳐 index 변경을 발견했습니다. 그 실패를 [V1](staged_publication_audit_v1.json)에 보존하고 index를 고정한 [V2](staged_publication_audit_v2.json)에서 **23.57초·PASS**를 확인했습니다. 10,453개 index entries에 gitlink·symlink·금지 경로·일반 100MiB 초과 blob은 없었고, 고유 텍스트 blob 5,960개 약 1.01GB에서 강한 비밀 패턴은 없었습니다. LFS pointer 2,864경로·고유 원본 2,522개 약 6.45GB의 로컬 바이트 해시는 모두 일치했습니다. 두 감사 보고서 자체와 이 설명은 감사 종료 뒤 별도 JSON·공백 검사로 추가하므로 그 V2의 전량 검사 분모에 포함하지 않습니다. 원격 LFS 업로드 성공과는 별개입니다.

## 모델 학습에서 실제로 진행한 부분

이번 새 학습은 **아직 시작하지 않았습니다.** 10월 12일 **17:36 KST** 재조회에서는 SSH 인증은 성공했지만 로드된 드라이버 580.173.02와 설치된 모듈·NVML 580.178.04가 달랐고 GPU 0 및 compute 조회 모두 종료값 **18**로 실패했습니다. GPU 유휴·가용 메모리·다른 GPU 작업은 확인하지 못했습니다. 서버 설치·재부팅·모듈 변경·GPU 1 사용은 하지 않았습니다.

**18:17 KST 최종 재확인에서도** SSH는 성공하고 GPU 0 및 compute 조회는 같은 NVML 불일치·종료값 18로 실패했습니다. 로드된 580.173.02와 NVML의 580.178 표시는 다시 확인했지만, 제한된 PATH에서 `modinfo`를 찾지 못한 오류도 별도로 남겼습니다. 이번 호출로 설치 모듈 버전을 다시 측정한 것은 아닙니다. [새 재확인 원본과 범위](remote_gpu_recheck_181722.json)를 보존합니다.

로컬에서는 [새 분할 처리 경계](../../../../../learning/portable_e2e/map_streaming_training_boundary_20261012_v1.py)와 [모의 검사](../../../../../learning/tests/test_map_streaming_training_boundary_20261012_v1.py)를 구현했습니다. 8배치 단위로 입력을 제한하고 reader를 마지막에 한 번 닫으며, optimizer 시도·반환·실제 ACK 반환·기록 완료를 분리합니다. 갱신 후 ACK 실패나 optimizer의 불명확한 부분 실패를 보존하고 첫 실패 뒤 재시도하지 않습니다.

ROOT 독립 재검사는 **25개 unittest methods 통과·11.396초**였습니다. 20,000행 모의 순서를 2,500개 묶음으로 처리했지만 **실제 TRAIN 재생·NN·손실·역전파·optimizer 갱신·CUDA 실행은 아닙니다.** 지연 public API와 봉인된 패키지 로더를 실제 수치 trainer에 연결하는 작업도 남아 있습니다. 외부 callback·전체 프로세스의 자원 소유권을 이 모의 검사만으로 보장하지 않습니다.

## 세 브랜치와 실차 준비

[세 브랜치 안내](../../../../autoware-three-branch-guide.md)에 실차 플랫폼용·CARLA E2E용·아키텍처 스캐폴드용을 구분했습니다. 플랫폼 모델·차량 인터페이스가 확인되지 않아 CAN/DBW나 실차 보정값은 추가하지 않았습니다. 스캐폴드 브랜치는 후속 변경의 기준선이며 이번 분리만으로 새 구조 변환이 완료된 것은 아닙니다.

주석 보완과 코드 게시가 자체 E2E 모델을 차량 제어에 승격하지 않습니다. 정식 완료는 **메인 0/9·하위 0/30**, 첫 RTE-01·SAF-01 지연 미달, 지도 기존 23/27·4개 실패·후보 미채택을 유지합니다.

## 다음 순서

1. 서버 관리자 조치 후 GPU 0 상태와 소유권 잠금을 재확인합니다.
2. 새 분할 경계를 실제 crop-only trainer·기록·패키지 로더에 연결하고 공통 초기 모델·optimizer·TRAIN/평가·종료 조건을 새 선언에서 검증합니다.
3. 같은 구조의 crop A/B 학습과 독립 평가를 수행합니다. 주행은 기본 기능의 실제 실패에 맞춘 별도 학습·폐루프 시험으로 이어갑니다.
4. 플랫폼 정보가 확인되면 비구동 메시지·센서 보정·통신 중단·명령 차단·비상정지부터 검증합니다. 실차 구동은 별도 승인 없이 하지 않습니다.

새 loader와 GPU의 실측 전에는 전체 학습 시간이나 공개 모델과의 성능 동등성을 보장하지 않습니다. [현재 전체 현황](../README.md), [원래 기능 로드맵](../../../../portable-e2e-feature-roadmap.md)을 유지합니다.
