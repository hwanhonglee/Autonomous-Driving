# 10월 2일 주행과 지도 모델의 현재 목표

<!-- HH_261012 - Apply the user's platform-name clarification without rewriting historical evidence or changing capability status. -->

**브랜치 용어 정정:** `fixmoving`은 앞으로 개발할 플랫폼 이름이며 차량 모델명이 아닙니다. 해당 브랜치는 플랫폼 개발 기준선으로 유지하고 구성·외부 인터페이스는 후속 설계에서 정합니다. 아래의 이전 실차 플랫폼 표현은 당시 해석이며, [현재 브랜치 안내](../../../autoware-three-branch-guide.md)를 기준으로 합니다. 이번 정정은 문서만 변경하며 새 학습·주행·기능 승인은 아닙니다.

<!-- HH_261012 - Prioritize the latest authenticated GPU failure and completed source publication preparation over older status snapshots. -->

**10월 12일 후속:** SSH 인증은 해결됐지만 17:36 KST의 GPU 0 조회는 NVIDIA 580.173.02/580.178.04 불일치로 종료값 18에 실패해 **새 학습은 시작하지 않았습니다.** 로컬에서는 Autoware 영어 주석 110개와 봉인 설정 설명 23개, 32개 exact 소스·4개 전체 patch를 준비했고 격리된 실제 복원에서 **68/68개 파일**을 재현했습니다. launch 회귀 373개·snapshot 검사 33개·새 분할 경계 모의 검사 25개가 통과했습니다. 20,000행 모의 검사는 실제 학습이 아닙니다. 실차 플랫폼·시뮬레이션·스캐폴드 브랜치의 용도와 미구현 사항을 분리하며 차량 구동·모델 승격은 하지 않습니다. [이번 결과와 다음 학습](../2026-10-12/03_autoware_publication_and_training_handoff/README.md), [세 브랜치 안내](../../../autoware-three-branch-guide.md)를 먼저 보세요. 아래는 이전 시점 기록입니다.

<!-- HH_261002 - Keep the nine main capabilities as the continuing objective rather than ending after a bounded straight precursor. -->

목표는 **자체 주행 모델의 메인 9개 영역과 하위 30개 기능**, 그리고 **온라인·주행 기록 기반 자동 지도 모델**이다. 현재 정식 완료는 **메인 0/9, 하위 0/30**이다. 작은 검사나 짧은 직진 성공을 메인 기능 완료로 세지 않는다. 기능 정의와 합격 기준은 [기존 기능 로드맵](../../../portable-e2e-feature-roadmap.md)을 유지한다.

## 최신 상태 — 10월 12일 CPU Tensor 입력 연결 검증 완료

<!-- HH_261012 - Separate restored remote access from the unresolved GPU driver mismatch and unimplemented full trainer. -->

**14:55 KST 후속 서버 확인:** SSH는 복구됐고 개인 venv·파일시스템 가용 약 2.99TB를 확인했습니다. 그러나 로드된 NVIDIA 580.173.02와 설치된 모듈/NVML/libcuda 580.178.04가 달라 깨끗한 환경의 GPU 0 조회도 종료값 18로 실패했습니다. **새 학습은 시작하지 않았으며** GPU 가용성을 확인할 수 없습니다. 공유 서버 관리자 확인 전 재부팅·모듈/드라이버 변경을 하지 않습니다. 새 지도 trainer·전체 20,000배치 연결도 아직 구현 전입니다. [실제 진단과 이어갈 조건](../2026-10-12/02_training_resume_preflight/README.md)을 먼저 보세요. 아래는 완료한 CPU 입력 단계의 근거입니다.

<!-- HH_261012 - Carry the ended actual CPU tensor parity and independent saved audit into the continuing two-model handoff. -->

지도 입력의 NumPy→Torch CPU 연결을 실제 데이터에서 검증했습니다. 원본 crop과 표지 보존 crop 각각 **16/16배치 통과·실패/미실행 0·같은 128회 입력**이며 원래 학습 코드의 CPU 변환과 RGB 바이트·정답 값이 전량 일치했습니다. 후보 정답 복구 2회가 Tensor·저장·실제 ACK까지 유지됐습니다. 실제 실행 **95.29초·관측 최대 약 369MiB·종료 seal 62개 일치**, 별도 코드의 지정 파일 **69개 저장 검산 9.78초**로 모두 종료됐습니다. 독립 검산은 저장 바이트·메타데이터이며 두 번째 배열/Torch 재생은 아닙니다.

**현재 학습 중인 작업은 없습니다.** 로컬 CPU 입력 검사이며 NN·손실·역전파·optimizer·PRO6000·GPU·CARLA·ROS·설치는 0회입니다. 정식 완료 **메인 0/9·서브 0/30**, 첫 RTE-01·SAF-01의 100ms 지연 미달과 지도 기존 23/27·4개 실패·미채택은 유지합니다. 같은 모델에서 crop만 바꾸는 학습 설계를 준비했고, 다음은 **새 trainer 입력 연결·전체 loader/기록/실패 조건 구현→개인 서버 환경/GPU0·새 TRAIN/평가 선언→학습·독립 평가**입니다. [읽기 쉬운 전체 현황·두 환경·다음 순서](../2026-10-12/README.md), [실제 입력 결과·코드·한계](../2026-10-12/01_map_torch_input/README.md)를 먼저 보세요. 아래는 이전 시점의 기록입니다.

## 10월 8일 입력 형식과 수신 확인 연결 완료

<!-- HH_261008 - Index the completed actual serial host boundary without calling the NumPy dry consumer a trained model. -->

**후속 실제 검증까지 완료했습니다.** 지도 입력을 float32 RGB·int64 정답으로 변환하고, 모든 값 검사→기록 저장→수신 확인(ACK)을 연결했습니다. 원본 crop과 표지 보존 방식 각각 **16개 통과·실패/미실행 0, 128회 입력**, 후보의 실제 복구 2회입니다. 전체 실행 **75.26초·관측 최대 약130MiB**, 종료 seal 41개 일치이며 별도 코드의 지정 파일57개 저장 검산도3.11초에 마쳤습니다.

**학습은 실행하지 않았고 현재 두 작업 모두 종료됐습니다.** Torch·실제 trainer 연결·전체20,000배치 실행은 아직 없습니다. 이번 작업은 로컬 CPU이며 PRO6000·GPU·CARLA·ROS·설치·모델 승격은0회입니다. 정식 완료 **메인0/9·서브0/30**, 첫 목표 RTE-01·SAF-01 지연 미달은 유지합니다. 다음은 기존 허용 환경의 Torch/trainer 경계 확인과 같은 구조 crop A/B 학습 계약입니다. [결과·완료와 미구현·다음 순서](../2026-10-08/nontraining_batch_capture/03_serial_training_input/README.md), [전체 현황](../2026-10-08/README.md)을 먼저 보세요.

## 10월 8일 13:09 실제 입력 복구와 읽기 개선 기록

<!-- HH_261008 - Carry the actual fixed-FIRST16 rescue/cache evidence into the handoff index without claiming trained capability or whole-run coverage. -->

**13:09 KST 현황:** 새 실제 첫 16배치에서 RGB·정답 복구 2회를 확인했습니다. 같은 V3 reader의 캐시 없음/사용 두 설정 모두 **16개 통과·실패/미실행 0·양쪽 바이트 결과 전부 일치**, 설정별 시간 **237.06→34.28초**, 반복 압축 해제량 약 **86.1% 감소**입니다. 전체 실행 274.11초·관측 최대 약 60MiB이며 실제 입력 진단 PNG를 생성·직접 확인했습니다. 독립 코드의 49개 모의 검사 후 지정 파일 68개·원래 20,000행과 선택 16행·두 전체 journal의 저장 검산을 3.09초에 완료했습니다. 독립 픽셀 재생이나 전체 20,000배치 RGB 검증은 아닙니다.

**새 학습·인식/주행 성능 향상은 아직 검증하지 않았습니다.** 로컬 CPU 작업이며 PRO6000 접속·GPU·CARLA·ROS·학습·모델 승격은 0회입니다. RTE-01 첫 목표·SAF-01 지연 미달·정식 완료 **0/9·0/30**은 유지합니다. 한 번의 순차 A/B 시간을 전체 학습 가속으로 확대하지 않습니다. 다음 비학습 작업은 새 revision의 trainer 입력 어댑터 구현과 순서·동일 영역·journal·실패 처리 검증입니다. 원격 학습은 접속·개인 hwanhong venv·물리 GPU 0 가용성 확인 후 별도 동일 구조 crop A/B로 이어갑니다. [실제 복구·PNG·측정과 한계·다음 순서](../2026-10-08/nontraining_batch_capture/02_actual_rescue_and_reader_ab/README.md), [오늘 전체 현황](../2026-10-08/README.md)을 먼저 보세요.

## 10월 8일 12:23 영상·정답 연결부 구현 기록

<!-- HH_261008 - Index actual limited RGB/GT processing and nonlive callbacks separately from historical target-only evidence and driving performance. -->

**12:23 KST 현황:** 학습 없이 지도 입력의 reader·동일 영역 crop worker·실제 바이트 journal writer를 구현했고, 실제 8배치가 **8개 통과·실패/미실행 0, 74.84초**에 종료했습니다. 과거 RGB·정답 해시를 둘 다 재현했으며 관측 최대 메모리는 약 55MB입니다. 별도 코드로 지정 파일 50개·원래 20,000행 메타데이터의 선택 8행·새 journal과 64개 원본 해시 witness도 저장 검산했습니다. 원래 나머지 19,992배치의 RGB 검증을 완료한 것은 아닙니다.

중요한 제한은 **선택한 64회 노출에서 실제 복구가 0회**였다는 점입니다. 배치 읽기·변경 없는 crop·바이트 기록을 실제 데이터에서 확인했지만, 복구 정책의 실제 RGB 효과나 모델 성능 향상까지 입증하지 않았습니다. 최종 합성 검사 185개와 별도 저장 검산용 스칼라 검사 20개는 각각 통과했습니다. 처음의 143개 통과·18개 실패와 중간 소스도 보존했습니다.

센서 수집 어댑터는 모의 callback으로 정상 2프레임과 깊이 응답 중단 시 실패 1·미실행 4프레임을 기록했습니다. 실제 CARLA 촬영·커널 소유권·파일 저장 인증·학습은 미실행입니다. **로컬 CPU 작업이며 PRO6000·GPU·다른 테스트의 시뮬레이터는 사용하지 않았습니다.** 첫 주행 목표 RTE-01, 처리 지연 SAF-01 미달, 정식 승인 **메인 0/9·서브 0/30**은 유지합니다.

다음은 실제 복구 조건의 RGB·정답을 새 선언으로 검증하고, 반복 압축 해제 비용을 줄인 loader 후보를 같은 바이트 기준으로 비교하는 것입니다. 원격 접속 복구 뒤에는 개인 hwanhong 환경·물리 GPU 0 가용성을 확인하고, 같은 구조·초기값·손실·optimizer에서 crop만 바꾸는 지도 A/B 학습으로 이어갑니다. trainer 연결과 새 학습은 아직 실행하지 않았습니다. [이번 구현·실제 결과·한계·다음 순서](../2026-10-08/nontraining_batch_capture/README.md), [오늘 전체 현황](../2026-10-08/README.md)을 먼저 확인하세요. 아래는 오전 9시까지의 이전 기록입니다.

## 10월 8일 오전 9시까지 비학습 개선

<!-- HH_261008 - Index the new nontraining source work before historical completed runs without claiming execution or approval. -->

**08:56 KST 현황:** 새 학습 없이 로컬에서 같은 모델의 실행 지연과 지도 정답을 검토했습니다. 정식 완료는 **메인0/9·서브0/30**, 첫 목표는 RTE-01 직진 차로 유지이며 지금은 SAF-01 실행 지연이 선행 문제입니다. 최신 CNN 컴파일 V21d는105.41초 후 지연 실패로 종료했고 독립 전량 검산을 마쳤습니다. 직접918개 출력은 V20과 전량 동일했지만 ROS27개 관측·891개 미실행, 완전한 궤적12쌍의100ms 통과는0쌍입니다. 같은12쌍의 ACK 중앙값104.03→104.31ms라 미채택합니다. 서로 다른 구간을 개선 추세로 합치지 않습니다.

지도는 실제350장·세 군1,050건의 정답 영역·픽셀 분석을 완료·독립 저장 검산했습니다. 기존23/27 통과·4개 실패·후보 미채택은 유지합니다. 이후 원래20,000배치·1,209개 슬롯을 전량 재검산한 새 정답 보존 재생에서 완전 잘림3,626→1,821회, 양성 중4.691→2.356%로 줄였습니다. 실제192.69초·실패/미실행0이며106개 파일·전체 통계·분모의 독립 저장 검산도 마쳤습니다. 표지 정답 픽셀 총량 증가1.10%이고, **입력 품질 개선과 모델 성능 개선은 다릅니다.** RGB배치/trainer 연결·새 학습은 아직 전입니다. 새 journal의59개 스칼라 검사·합성20,000행 전량 검증도 완료했습니다. [결과와 다음 동일 구조 A/B 설계](../2026-10-08/mapping_review/crop_preservation/README.md)를 확인하세요.

**차량 중심 기록 화면을 실제 완료했습니다.** V9는38.39초에 DIRECT918+ROS29의 전체947프레임 GIF 두 개와PNG3장·전체1,836행CSV를 생성했습니다. 원래ROS 실패1·889개 미실행을 유지하며, 독립 전체 저장·형식 검산과 ROOT고정PNG3장 열람도 완료했습니다. 실제 합11,336,666바이트로 기존12MiB GIF 한도 안입니다. [화면·GIF·범위 설명](../2026-10-08/recorded_visuals/README.md)을 보세요. 기록 진단 그림이며 새주행·RViz·실차 영상은 아닙니다. 최근 종료 상태 명령도103개 사전 검사와 새 스냅샷의 실제 명령·독립28개 메타데이터 검산을 완료했습니다. 네 종료 결과와28개 선언값을 확인했으며 서버 전체 현재 실행 여부를 조사한 명령은 아닙니다. **이번에는 새 학습을 시작하지 않았습니다.** 08:31–08:32 SSH재확인도 인증 전 `No route to host`였고 백업 전 삭제0개입니다. 로컬 레거시 별칭·workflow 바로가기0개, 새 문서 누락0개이며 과거 고정 자료 누락1건은 남아 있습니다. [최신 전체 현황·다음 순서](../2026-10-08/README.md), [실제 주행 지연](../2026-10-08/driving_ros/README.md), [두 모델의 실제 학습 범위](../2026-10-08/model_scope/README.md)를 먼저 확인하세요. 아래는 이전 시점의 기록입니다.

## 10월 7일 학습 없이 연결과 지도 검증

<!-- HH_261007 - Index real nontraining checks and the preserved coordinate-check failure separately from source preparation and capability approval. -->

현재는 새 학습 없이 기존 모델의 Autoware 기록 입력 연결과 지도 좌표·누적을 검증합니다. CPU 환경과 Autoware 메시지형 10종의 직렬화·복원, 실제 작은 메시지 **6/6건의 내용·송신자 확인**을 완료했습니다. V8·V9·V10은 각각 **직접 918/918개 완료** 후 실제 DDS 지연 관문에서 실패·종료했습니다. V9의 완전한 출력 35쌍 중 전체 ACK 100ms 충족은 **8쌍**, 검사 위치 변경 V10의 29쌍 중 **5쌍**이라 개선 근거가 없고 채택하지 않습니다. 마지막 부분 Trajectory와 V9 **868개**, V10 **873개 미실행**도 보존합니다. 현재는 같은 NN·안전·100ms를 유지하며 실제 처리 단계의 비용을 더 세분화합니다. 학습이나 실제 차량 주행이 아닙니다.

지도는 역변환 버그를 고친 V2에서 **480프레임·누적 상태 1,440개·최종 격자 6개**의 일치를 확인했고, 최신 저장 마스크 **1,050/1,050건**을 분석·독립 검산했습니다. 후보가 표지 오탐은 줄이지만 누락이 늘어 기존 **23/27 통과·4개 실패**를 유지합니다. 좌표 검산이나 작은 송수신 성공을 주행 기능·정밀지도 성능의 합격으로 세지 않습니다. [전체 현재 상태·9개 영역·실제 지연 그래프·다음 과제](../2026-10-07/README.md)에 구분했습니다.

추가 V11은 **실제 43묶음·817메시지·8,170단계** 계측을 완료하고 종료했습니다. 완전한 26쌍 중 전체 ACK 적시는6쌍뿐이며 마지막 Trajectory가100.65ms로 거절돼875개 미실행을 유지합니다. native 수신/조회19회의 합은 묶음당평균1.70ms, 소유 프로세스 확인은4.62ms로 관측됐습니다. 전체 입력 준비32.71ms를 순수 DDS 지연으로 설명하지 않습니다. 지도는 원래 소스15개·350장 참조 연결 검토와 평가crop 부재 확인 후, **원본 RGB/semantic↔package의 실제350장 픽셀 전량 일치**를4.56초에 확인했습니다. 실패·미실행0이며 기존V1의 실행 전 용량 중단도 보존했습니다. 픽셀 일치를 센서 시각·객체 정답성·모델 인식 개선으로 세지 않습니다. [현재 지도 검사·해석·남은 과제](../2026-10-07/mapping_review/README.md)를 따릅니다. 새 학습이나 주행 기능 승인은 아닙니다.

**첫 목표RTE-01·정식 완료0/9·0/30·새 학습0회**입니다. GPU 준비와 새 폐루프는 공간 조건이 아직 미달이며, 다른 테스트의 CARLA는 사용하지 않습니다. [읽기 쉬운 현재 작업·실제 실패·다음 순서](../2026-10-07/README.md)를 먼저 확인하면 됩니다. 아래는 이전 시점의 자료 정리 기록입니다.

## 10월 7일 논문 자료 분류와 공간 조사

<!-- HH_261007 - Index completed actual home relocation and full preservation without treating organization as recovered capacity or model progress. -->

**10월 7일 16:31 KST:** Documents·Downloads 자료를 홈의 [thesis_4pc 안내](/home/a/thesis_4pc/README.md)로 **실제 분류 이동**했다. 26개 상위 항목에 포함된 일반파일154개, 총2,593,062,825byte 약2.59GB이며 전체 SHA256·inode·소유자·권한·mtime을 다시 확인했다. Documents에 남긴 Unreal Projects·AnyDesk 두 폴더의 내용도 그대로다. 논문, PC4 문서, 원본 시험, 분석, 지도 원본, 설치 파일, 이동 기록의7개 분류다. FBX·텍스처의 위치 관계와 원본 Chrony 체크섬을 보존했다. 설치된 환경과 실행용 지도·고정4-PC 원본은 유지했다.

이동·원복 임시 폴더 검사18개가 통과했고, 앞선 목록 검사4개와 합쳐 **22개 검사 통과**다. 큰 파일 복사0·영구 삭제0·레거시 링크 생성0·학습/GPU/CARLA 실행0이다. 같은 디스크 안의 이동이므로 여유 공간을 확보했다고 주장하지 않는다. [원래 위치·새 위치·전체 해시·원복 방법](/home/a/thesis_4pc/90_move_records_and_cleanup/README.md)을 따른다. 첫 주행 목표 RTE-01과 정식 완료0/9·0/30은 그대로다. 아래는 앞선 시점 기록이다.

<!-- HH_261007 - Record completed metadata organization separately from deletion, live simulation, learning and capability approval. -->

**10월7일:** 사용자의 우선순위에 따라 학습 대신 PC4대 졸업논문 자료와 PC 공간을 조사했다. [새 통합 분류 폴더](../../../thesis_4pc/README.md)에서 시스템·이력, 준비, 원본, 논문 분석, 재현, 용량 후보를 찾을 수 있다. 메타데이터 목록 도구의4개 검사는 통과했고 원본 이동0·삭제0이며, CARLA·Unreal·Autoware·RoadRunner·지도·모델·venv·실패 기록을 보존한다. 공개PC1–4자료약1.22GB, 별도PC4원본약2.01GB, 2025자료약0.55GB이다. 이 VILS 자료는 portable E2E 기능 승인과 별개다.

쓰기 가능공간은15시전후약4.47GB다. 오래된 ROS로그약20.60GB와 휴지통의일반파일약16.06GB를 새 후보로 찾았으나 보존 범위·최종비사용·영구삭제승인 전 제거하지 않는다. 휴지통의전체하드링크별칭은조사범위안에있지만 이것은비사용증명이아니다. PC1실패근거로직접참조된ROS로그도보존한다. 기존캐시정리승인은 유지하며, 오늘접속도 `No route to host`여서 서버백업은 미진행이다. 다음은 사용자선택에 맞춘 원본이동 범위 확정·검증된 공간정리→동일모델의지연·Autoware비구동연결→실제직진·독립반복이다. 이번작업에서 학습·GPU추론·CARLA실행은0이며 첫목표RTE-01, 공식완료0/9·0/30이다. 아래는 이전 시점 기록이다.

<!-- HH_261006 - Apply existing approval, record the actual failed remote backup, and forbid unverified deletion or a false running/training claim. -->

**10월 6일 12:13 KST: 정리 승인은 반영했고 실제 작업을 시작했다. 현재 막힌 것은 승인이 아니라 서버 접속이다.** 삭제 안전 테스트30개를 통과한 뒤 `/var/tmp/ccache`의 M/R42,275개를 전량 해시 검사했다. 로컬 참조·미분류 위험은0개이며 통계/표식288개·폴더273개를 별도로 보존했다. 권한 때문에 읽지 못한 기존 시스템 서비스의 FD를 비사용 증명으로 주장하지 않는다. 실제 서버 백업은 `220.90.5.14:6000`의 **인증 전 `No route to host`**로 실패했다. 별도 읽기 전용 접속도 같은 오류였고 기존 승인된 우회 접속은 찾지 못했다. 서버가 꺼졌거나 비밀번호가 틀렸다고 단정하지 않는다.

**삭제0개·검증된 서버 백업 없음·새 GPU 계산/주행/학습0회**다. 실패 영수증의 기존 `remote_backup_retained:true` 필드는 실제 원격 백업 증거가 아니며 채택하지 않는다. 독립 읽기 전용 검산에서 전체42,275개와 보존288개·273폴더의 세대/바이트 해시·소스가 검사 당시와 동일함을 확인했다. 로컬 쓰기 가능 약5.28GB는 다음 GPU 준비의 기존 예약약5.97GB에 미달한다. 백업 없이 삭제하거나 예약 기준을 낮추지 않는다. 서버 전원·포트·기관/VPN 접속 조건을 확인한 뒤 **원래 실패를 보존한 새 백업 시도→전량 검증→승인된 캐시 개별 제거→동일 GOOD 모델 GPU 출력/시간 비교→실제 직진·독립 반복**으로 이어간다. 학습은 이미 종료됐고 첫 목표RTE-01, 정식 완료는 메인0/9·서브0/30이다.

[실제 검사·백업 승인·접속 실패 원본](../../../../storage/experiments/maintenance/2026-10-06/ccache_mr_v1/), [전량 보존 독립 검산](../../../../storage/experiments/maintenance/2026-10-06/ccache_mr_v1/ROOT_failed_backup_whole_local_postcheck_v1.json), [통과30개·첫 수집 실패를 포함한 테스트 기록](../../../../storage/experiments/plans/2026-10-06/ccache_cleanup/). 아래는 당시 시점의 기록이다.

<!-- HH_261006 - Recheck external state after repeated approval without asking again or retrying frozen backup outputs. -->

**12:15 KST 재확인:** 같은 정리에 대한 추가 승인은 필요 없다. 실제 SSH를 한 번 다시 확인했지만 여전히 인증 전 `No route to host`였다. [전체 접속 명령·출력·시각](../../../../storage/experiments/maintenance/2026-10-06/ccache_mr_v1/ROOT_connection_recheck_v1.json)을 새 기록에 보존했다. 별도 로컬 확인에서는 runtime 잠금 사용 가능·CARLA0개·GPU compute0개였으나 가용5.263GB가 기존 GPU 준비 요구5.973GB에0.710GB 미달했다. 접속 복구 전 백업·제거·GPU 준비·학습은 실행 중이 아니다. 승인 대기가 아니라 외부 접속 상태 확인이 필요한 상황이다.

<!-- HH_261006 - Record reaffirmed cache cleanup approval and distinguish preparation from backup, deletion and GPU execution. -->

**10월 6일 11:55 KST:** 사용자의 재확인을 추가 컴파일 캐시 백업·검증 후 정리 승인으로 반영했다. 동일 승인을 다시 요청하지 않는다. 현재 `/var/tmp/ccache`의 본인 M/R 데이터42,275개·할당약1.80GB를 새로 확인했고, 통계·표식·폴더·설치·venv·소스·학습/주행 자료는 유지한다. **백업·삭제·GPU 계산은 아직 시작 전**이며 새 검증된 절차를 준비한다. 로컬 GPU0에는 기존화면4개 프로세스만 있고 compute는0개였다. 공간 확보 후 원래 가중치의GPU CNN 변환→전체 출력 동등성·시간 비교→실제 직진·독립 반복 순서다. 학습은 이미 끝났고 첫 목표RTE-01, 정식 완료0/9·0/30이다. [추가 승인 범위](../../../../storage/experiments/plans/2026-10-06/ccache_cleanup/user_scope_approval.json), [이전 실제 결과·후속 설계](../2026-10-05/README.md)를 구분한다. 아래는 당시 기록이다.

<!-- HH_261005 - Make the ended learning, rejected redundant optimization and pending extra cleanup authority the next handoff entry. -->

**10월 5일 12:27 KST:** 학습3개·5모델35,315개 평가·497개 원래 판정의 독립 검산을 끝냈으며 현재 학습/GPU 평가 작업은 없다. 새3개 모두 HOLDOUT 경로 오차가 기존 GOOD보다 커져 채택하지 않았다. 새 해시 캐시 가설도 실제883회에서 이미 캐시가 적용된 것을 확인하여 NN 비교 전에 제외했다. 첫 목표 **RTE-01**, 정식 완료 **0/9·0/30**이다. 승인된 `.o` 정리는 완료했고, 추가 컴파일 캐시 정리는 별도 승인 대기다. 공간 확보→동일 모델 GPU 출력 동등성·속도→실제 직진·독립 반복 순서이며 [읽기 쉬운 전체 현황·후속 학습 설계](../2026-10-05/README.md)를 따른다. 아래는 당시 시점의 기록이다.

<!-- HH_261005 - Index the ended fixed-five evaluation, whole verified failure denominator and continuing runtime work. -->

**10월 5일 12:10 KST:** 새3개 모델의 학습과 기존2개+새3개 **전체35,315개 평가·독립 전량검산이 완료**했다. 현재 학습/GPU 평가 작업은 없다. 원래497개 비교 중64개 미달이며 새 모델을 자동 채택하지 않는다. 첫 목표 **직진 차로 유지(RTE-01)**, 정식 완료 **메인0/9·하위0/30**이다. 동일 기존 모델의 입력 처리·추론 지연 개선을 이어가며 추가 공간 확보 전 새 실주행은 미실행이다. [현재 결론·다음 순서·읽기 쉬운 결과](../2026-10-05/README.md)를 먼저 읽으면 된다. 아래 시각별 기록은 당시 상태다.

<!-- HH_261005 - Distinguish completed optimizer fits from the actually running fixed five-model GPU0 evaluation. -->

**10월 5일 11:31 KST:** 새 주행 모델 3개의 학습과 전량 사후검산은 모두 완료했다. 지금은 **PRO6000 GPU0에서 기존 2개와 새 3개 모델의 실제 평가가 실행 중**이다. 첫 모델의 7,063개 평가를 끝내고 두 번째 모델로 진행했으며, 전체 예정 분모는 **35,315개**다. 최종 비교 판정은 아직 없다. 로컬에서는 현재 모델의 비구동 Autoware 연결과 추론 지연 개선 코드를 준비한다. 새 실주행은 추가 쓰기 공간이 필요해 아직 실행하지 않았다. 첫 목표는 **직진 차로 유지(RTE-01)**, 정식 완료는 **메인 0/9·하위 0/30**이다. [현재 작업과 다음 순서](../2026-10-04/README.md), [실제 평가 실행 승인](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_final_evaluation_20261005_v1/ROOT_final_evaluation_execution_review_v2.json). 아래 시각별 기록은 당시 상태다.

<!-- HH_261005 - Record completion of all three actual fixed6000 fits and preserve a rejected ROOT serialization before evaluation. -->

**10월 5일 11:14 KST 확인:** 새3개 모델의 **각6,000회·입력24,000개 학습과 전량 사후검산은 모두 완료**했다. 마지막 학습 계산은약31분57초였다. 현재 학습 중인 작업은 없다. 실제5모델 평가 전 ROOT가 승인 JSON의`5.0`을`5`로 저장해 원본 학습 조건 해시가 달라지는 오류를 냈다. 검사에서 거부돼 **원격 전송·실제평가0회**이며, 실패V1을 보존하고 원본 자료형을 유지하는 새V2 승인·감시 연결을 준비한다. 최종 비교→지연 개선→실제 직진·독립 반복 순서이고, 정식 완료는 여전히 **0/9·0/30**이다. [실제 세 번째 전량검산](../../../../storage/experiments/plans/2026-10-05/temporal_route_joint_three_seed_train_only_v2/seed_2026100503_fit_supervisor_v2/ROOT_whole_seed3_training_postcheck_v1.json), [현재 작업·남은 문제](../2026-10-04/README.md). 아래 시각별 기록은 당시 상태다.

<!-- HH_261005 - Publish real optimizer progress and the preserved failed supervisor draft without claiming a driving capability. -->

**10월 5일 10:59 KST 확인:** 첫 두 모델의 학습·전량검산은 완료했고, 세 번째 모델은 **실제 학습 중**이다. 최근 확인은 **4,400/6,000회·입력17,600개·오류 로그0**이다. 평가 실행 코드의 첫 모의 검사에서는 실제 실행 구문 선택 오류로 **39개 통과·3개 실패**가 나왔고 원본을 보존했다. 다른 학습 번호의 모델 파일을 섞지 못하도록 연결 검사도 보강 중이다. 실제 5모델 평가·새 로컬 주행은 아직 시작하지 않았다. 첫 목표는 **직진 차로 유지(RTE-01)**이며 정식 완료는 **0/9·0/30**이다. [읽기 쉬운 현재 상태와 다음 순서](../2026-10-04/README.md). 아래 시각별 기록은 당시 상태다.

<!-- HH_261005 - Index two completed actual fits and the newly started third process; retain the raw/canonical postcheck mistake and deny unmeasured capability claims. -->

**10월 5일 10:32 KST:** 두 모델은 각각 **6,000회·입력24,000개** 학습과 전량 사후검산을 완료했다. 두 번째 학습은 **31분33초**였다. 두 번째 검산의 첫 시도는 ROOT가 JSON 내용 해시를 원본 파일 바이트 해시로 잘못 승인해 거부됐다. 그 실패를 보존하고 별도V2·올바른 원본 해시로 다시 검사하여 전체6,000행·12개 체크포인트·최종101개 상태/동결47개·TRAIN 영상34,992개·원본과 소스·종료를 확인했다. **PRO6000 GPU0의 세 번째 프로세스가10:30:43에 시작됐으며 초기 검사 중**이다. 마지막 확인은 stdout/stderr0으로 optimizer 업데이트는 아직 확인하지 않았다. 로컬 새 GPU 추론·실주행은 공간 부족으로 미실행이다. 최종 비교는 공통3000·기존GOOD6000·새3개 최종6000 모델 모두와 기존71기준·전체35,315평가 기록을 유지해 준비 중이며 아직 실제 평가하지 않았다. 첫 목표 **RTE-01 직진 차로 유지**, 정식 완료 **0/9·0/30**을 유지한다. [원본·해시 실수와 수정·현황·다음 순서](../2026-10-04/README.md). 아래는 각 시점의 과거 기록이다.

<!-- HH_261005 - Distinguish the completed and independently checked first fit from the actually running second optimizer. -->

**10월 5일 09:42 KST:** 첫 번째 학습은 **6,000회·입력24,000개·약30.9분**으로 종료했다. 전체6,000행·12개 체크포인트·최종101개 상태/동결47개·TRAIN 영상34,992개·원본과 실패 보존의 독립 사후검산도 통과했다. **PRO6000의 개인 venv·물리GPU0에서 두 번째 모델을 실제 학습 중**이며 최근 확인은 **600/6,000회·입력2,400개**, 오류 로그0이다. 같은 승인된 TRAIN·공통 시작 가중치·새 optimizer를 사용하고 학습 순서만 다르다. 첫 모델을 이어 학습하거나 자동 채택한 것이 아니다. 로컬 새 실주행/GPU 추론은 공간 부족으로 대기하며 지도 새 학습은 실행하지 않는다. 다음은 두 번째 종료·전량검산 → 세 번째 순차 학습 → 최종 모델 평가다. 첫 학습을 참고한 학습 자체 예상은 약31–33분/모델이며 후속 평가·검산 시간은 별도다. 학습 완료는 실제 주행 성공이 아니다. 첫 목표 **직진 차로 유지(RTE-01)**, 정식 완료 **0/9·0/30**이다. [실행 근거·전체 현황·다음 순서](../2026-10-04/README.md). 아래는 각 시점의 과거 기록이다.

<!-- HH_261005 - Record completed actual TRAIN admission separately from the not-yet-started GPU fit and blocked local runtime. -->

**10월 5일 08:46 KST:** PRO6000의 실제 CPU 학습 입장 검사와 전량 사후검산은 완료했다. 승인된 TRAIN만 사용하며 3개 서로 다른 학습 순서, 공통 101개 상태, 소스 33개를 확인했다. **GPU 학습은 아직 시작 전**이며 별도 GPU0 승인·실행 감시 코드의 마지막 검토 중이다. 로컬 GPU 준비 V6는 컴파일 후 소유권 조회 불일치로 종료했고, 실제 추론은 하지 못했다. 새 V7의 모의 검사 424개는 통과했으나 현재 쓰기 가능 약4.80GB로 실제 실행 예약에 미달한다. 현재 모델의 Autoware 연결도 준비 코어만 있으며 실제 ROS 연결·동등성은 미구현/미검증이다. 첫 목표는 **RTE-01 직진 차로 유지**, 정식 완료는 **0/9·0/30**이다. [전체 현재 상태·CPU 입장 원본·다음 작업](../2026-10-04/README.md). 아래 시간 표시는 이전 기록이다.

<!-- HH_261005 - Index the running recorded CPU comparison and actual angular setup failure without claiming ongoing learning or feature completion. -->

**10월 5일 08:12 KST:** 로컬 OpenBLAS 1/6 비교는 전체1,776개·60계측 독립 확인 후 종료했으며 평균32.44/32.94ms로 실제 가속 근거가 없다. 정지 V6도 원래 FAIL·60미실행을 보존했다. 같은 CNN GPU 준비 V5는 C++ 문자열 출력 함수명 충돌로 컴파일 단계에서 종료했다. 실패 원본·63개 전체 파일·53개 원본/보존 소스·모델 불변을 독립 확인하고 함수명만 수정한 새 V6를 준비한다. CUDA/GPU build·추론·새 실주행은 미실행이다. PRO6000 학습은 아직 시작하지 않았으며 새3-seed 코드의113개 모의 검사를 통과했다. 준비 승인과 학습 승인 파일을 분리하는 새 버전을 먼저 확인한다. 첫 목표 RTE-01, 정식 완료 **0/9·0/30**을 유지한다. [현재 작업·전체 실제 실패·다음 순서](../2026-10-04/README.md). 아래 시간은 이전 기록이다.

<!-- HH_261005 - Keep ended no-gain and pre-capture invocation failures visible beside independently verified model export feasibility. -->

**10월 5일 07:14 KST:** CPU 코어 비교4조건은 종료·미채택이며 독립 전량 검산 중이다. 정지V4는 ROOT의 실제 명령 경로 불일치로 계측 전에 종료했고 실패·소유 서버 정리·전체 원본을 확인했다. 같은 CNN의 CPU 변환과 두 파일의14개 가중치·고정 입력/출력 독립 확인은 통과했지만 GPU 가속은 아직 미검증이다. 새V5 정지 진단과 GPU 실행 가능성 검사를 준비 중이다. 학습·주행·GPU 작업은 현재 실행 중이 아니며 **0/9·0/30**이다. [현재 결과·원인·다음 순서](../2026-10-04/README.md).

<!-- HH_261005 - Preserve adverse preliminary timing and the current temporal Autoware integration gap without increasing capability approval. -->

**10월 5일 07:03 KST:** 로컬 CPU 비교4조건 중3조건이 종료하고 마지막A2를 실행 중이다. 먼저 끝난 A1·B1의 평균229.79ms·291.51ms는 개선 근거가 아니며, 전체 출력·시간의 독립 검산 뒤에만 판정한다. 현재 GOOD6000과 기존 Autoware shadow 코드의 입력·모델 ABI 차이도 확인했다. 별도 연결·동등성 검증이 남고, 이전 VAD 화면을 현재 모델의 성공으로 세지 않는다. PRO6000 학습은 실행 중이 아니며 정식 완료 **0/9·0/30**이다. [현재 실제 결과와 다음 순서](../2026-10-04/README.md).

아래 시간 표시는 이전 진행 기록이다.

<!-- HH_261005 - Distinguish a running readonly CPU comparison from ended conversion and stationary setup, with no approved feature increase. -->

**10월 5일 06:50 KST:** 로컬에서 GOOD6000의 CPU 코어 배치 비교를 실제 실행 중이다. 같은 CNN의 CPU 변환은 두 입력 조건 모두 종료·통과했지만 GPU 속도·출력 동등성은 미검증이다. 정지 계측 V3은 실제 Town03·맵 내용 확인 후 초기 정지 속도 검사에서 종료했고, 최초 실제값을 기록하는 V4만 준비 중이다. PRO6000 학습은 실행 중이 아니다. 정식 완료는 여전히 **0/9·0/30**이며 [최신 결과·한계·다음 순서](../2026-10-04/README.md)를 따른다.

<!-- HH_261005 - Keep the latest actual stationary setup outcome distinct from prepared acceleration and ended training. -->

**10월 5일 06:29 KST:** 정상 라이브러리를 사용한 정지 계측 V2는 실제 로딩·연결 후 맵 검사에서 종료했다. 비교 0회이며 실패 분모 60회를 그대로 보존했다. 새로 만든 소유 서버만 Town03으로 초기화하는 별도 V3을 준비 중이다. 실제 V3·GPU 가속은 아직 실행하지 않았고 PRO6000 학습도 실행 중이 아니다. [최신 원인·검증·다음 순서](../2026-10-04/README.md)를 따른다. 주행 기능 승인은 **0/9·0/30**이다.

<!-- HH_261005 - Keep the handoff index current without converting a known-DEV straight drive into independent feature approval. -->

실제 209m 직진 시험에서 목표 도달·충돌 0·차체 침범 0을 확인했다. 하지만 처리 속도 **8.62Hz**와 최대 명령 확인 지연 **109.60ms**가 원 기준에 미달해 종합 판정은 **FAIL**이다. 첫 목표는 여전히 **RTE-01 직진 차로 유지**이며, 정식 완료는 **메인 0/9·하위 0/30**이다.

승인된 빌드 중간 파일 5,496개는 서버 개인 폴더에 백업·검증 후 정리했다. 현재는 로컬에서 바퀴 각도 조회의 정지 차량 비교를 준비 중이다. **PRO6000 학습은 실행 중이 아니다.** 지도 모델은 별도 개발하되 첫 주행 기능의 안정화가 우선이다. [현재 결과·전체 실제 화면·다음 순서](../2026-10-04/README.md)를 따른다. 아래 10월 2일의 학습 중 표시는 당시 기록이다.

<!-- HH_261005 - Retain the failed stationary diagnostic and keep the current source-only work distinct from running learning. -->

추가 정지 차량 진단 V1은 계측 전 SIGBUS로 종료했고 서버·포트 정리는 확인했다. 실제 자식의 전용 라이브러리가16MiB에서 잘린 사실을 확인했다. 이후 기존 파일 제한을 유지한 **서버 연결 없는 실제 로딩 진단은 정상 종료**했다. 현재는 정상 캐시의 새 정지 차량 비교와 같은 모델의 GPU 실행 후보를 준비 중이며 실제 주행 지연 이득은 아직 미검증이다. 기능 완료 수는 늘리지 않는다. 실패와 후속 상태는 위 최신 기록에 계속 남긴다.

## 10월 4일로 이어진 이전 기록

<!-- HH_261004 - Supersede stale running claims with completed balanced20k evidence and a fresh live-status index. -->

[현재 목표·주행 보존 비교 학습·다음 Town03 직진 시험](../2026-10-04/README.md)으로 이어갑니다. 아래 10월 2일의 진행 시점은 과거 기록입니다. 좌우회전 균형 학습은 10월 2일 **21:42:32 KST에 전체 평가까지 종료**됐으며, 20,000회·100,000개 입력 노출을 완료했습니다. 같은 기존 검사 74개는 이전 60개 통과에서 **59개로 퇴행**, 새 우회전 TRAIN 검사 3개를 포함한 전체는 **62/77**입니다. 기존 실패 14개도 남아 **미채택**입니다. 현재 실행 상태와 이 과거 학습을 혼동하지 않습니다.

## 10월 2일 19시 38분 당시 기록

<!-- HH_261002 - Put the actual new GPU0 optimizer run before the preserved paired failures and nine-domain table. -->

- **PRO6000: 새 좌우회전 균형 모델을 실제 학습 중.** 지도 학습·전체 평가 종료와 GPU0 비어 있음을 확인한 뒤, 19:05:51에 새 프로세스를 실행하고 19:06:29에 실제 optimizer 학습을 시작했다. 19시 38분 확인은 **4,638/20,000회·23,190/100,000개 입력 노출**, 오류 로그 없음이다. 원래 6,000회 모델에서 새 optimizer로 시작하며 정상 LEFT347·RIGHT697과 기존 직진·복귀를 함께 학습한다. 같은 101개 가중치 구조와 기존 보호 손실·74개 진단 기준·실패 14개를 유지한다. ROOT·독립 코드 검사, 실제 TRAIN CPU·5개 CUDA gradient·원본 카메라 41,256개·소스 31개를 확인했다. 처음 속도 기준 학습 약 2~3시간과 추가 평가로 예상하며, 좋은 중간 모델을 고르거나 자동 배포하지 않는다. [진행 상태·범위·다음 실제 주행](portable_e2e_learning_cycle_v1/26_actual_left_right_joint_learning/README.md).
- **PRO6000: 새 주행 모델의 20,000회 학습·전체 평가 완료, 미채택.** GPU0·개인 `py312` venv에서 80,000회 입력 노출을 완료했다. 실제 학습 102.74분·평가 18.22분이며, **74개 중 60개 통과·14개 미달**이다. 같은 좌회전 TRAIN 347개의 미래 위치 오차는 **0.347→0.062m**로 줄었지만 기존 저속 위치·복귀 속도 개선·Town03 우회전 속도 보호가 미달했다. 학습 소스 30개·모든 최종 평가·20개 체크포인트·20,000개 업데이트를 보존하고 서버 전후·로컬 전량 동일성을 확인했다. [같은 자료의 비교표·미달 항목·전체 원본](portable_e2e_learning_cycle_v1/20_actual_low_speed_left_joint_learning/README.md).
- **로컬: 새 모델의 실제 좌회전 완료, 정확도는 미달.** 기존 모델은 차선 침범 1회로 약 30초에서 수집 종료됐다. 새 모델은 **89.13° 회전·출구 뒤 10.70m·3초 안정 주행**, 모델 제어 이후 91.89m 진행과 충돌·침범 0회, 차체 여유·100ms 처리를 통과했다. 그러나 중심 오차 p95 **0.413m > 기존 0.35m**여서 종합 판정은 FAIL이다. 같은 처음 513개 상태의 오차는 0.787→0.427m이며, 전체 분모가 다른 두 시도를 같은 길이처럼 비교하지 않는다. [차량 중심 전체 GIF·PNG와 실제 비교](portable_e2e_learning_cycle_v1/23_actual_left_joint_left45_comparison/README.md). 이후 같은 새 가중치의 추종 거리 A/B도 실제 완료했으며 **0.414→0.386m**로 줄었지만 둘 다 같은 0.35m에 미달했다. [새 두 조건 전체 결과](portable_e2e_learning_cycle_v1/24_actual_left_joint_lateral_lookahead_pair/OUTCOME.md). TRAIN 경로의 성공을 독립 기능 승인·정차·30km/h로 세지 않는다.
- **PRO6000: 두 주행 모델 학습·전체 평가 완료.** GPU0·기존 개인 venv에서 대조군·회전 경로 손실 후보 각각 **6,000회·24,000회 입력 노출**을 완료했다. 학습 약 30.3분·29.8분, 두 최종 모델 뒤의 전체 평가 약 27.2분이다. 모든 체크포인트·12,000개 실제 업데이트 기록·18개 평가 파일을 보존하고 로컬 전량 SHA 검증도 완료했다. 회전 TRAIN 경로 손실은 대조군 1.802 → 후보 0.835로 약 54% 낮아졌지만, 기존 진단은 **59/71 → 54/71**로 퇴행했다. **후보를 채택하지 않았으며 오프라인 오차 감소를 실제 회전 성공으로 세지 않는다.** [전체 실제 평가](../../../../storage/experiments/training/2026-10-02/temporal_intrinsic_curve_pair_v2/summary.json).
- **로컬: 두 새 모델의 실제 우회전 완료, 둘 다 실패.** 같은 전체 경로·110초·CPU4·PI2.4·0.5초 설정에서 대조군은 약 **48°**, 후보는 약 **38°**만 회전하고 경로 이탈로 안전 제동됐다. 예정 약 91° 회전·회전 후 복귀는 둘 다 완료하지 못했다. 최대 관측→명령 확인 지연은 **82.11·90.25ms**로 100ms 안이었지만, 전체 실제 급감속 기준은 둘 다 미달이다. 실패 꼬리를 포함한 상태 2,200개·카메라 묶음 1,100개를 각각 보존한다. [대조군 원본](../../../../storage/experiments/validation/2026-10-02/temporal_intrinsic_pair_town03_known_dev_right_v1/control/diagnostic_report.json), [후보 원본](../../../../storage/experiments/validation/2026-10-02/temporal_intrinsic_pair_town03_known_dev_right_v1/intrinsic_curve/diagnostic_report.json). [전체 차량 중심 GIF·PNG와 읽기 쉬운 비교](portable_e2e_learning_cycle_v1/17_intrinsic_curve_pair_actual_town03_right/README.md)를 게시하고 입력·소스 10,461개와 미디어 6개씩을 다시 검증했다. 직진 보호 6회는 초기 전량 저장 공간 예약이 아직 부족해 실행하지 않았다. 현재 모델 학습을 먼저 진행한다.
- **지도: 새 표지↔배경 손실 비교도 실제 학습·평가 완료, 미채택.** GPU0·개인 venv에서 각각 **4,000회·32,000장 노출**을 완료했다. 실제 학습 시간은 대조군 약 4.5분·후보 약 5.8분이다. Town01 배경→표지 오탐은 **25,548→9,277**로 줄었지만, 표지 recall은 **63.81→39.59%**, 차선 IoU는 **97.51→96.42%**로 떨어졌다. 전체 고정 350장·27개 판정은 **21개 통과·6개 실패**다. 53개 결과 파일을 서버 전후·로컬 전량 SHA로 검증했고, 기존 모델은 교체하지 않는다. [전체 실제 수치·실패 6개](../../../../storage/experiments/training/2026-10-02/map_sign_other_margin_repair_v1/summary.json).
- **완료 수: 메인 0/9·세부 0/30.** 현재 우선 목표 5개는 직진·좌회전·우회전·목적지 정지·속도 유지다. 기능 head를 더 붙이기 전에 실제 기본 주행부터 확보한다. 30km/h는 그다음, 60km/h는 후순위다.
- **지도: 구조 비교 v3의 두 학습·350장 전체 평가·독립 재검산 완료, 미채택.** 두 구조 각각 **20,000회·160,000개 노출**을 완료했다. 실제 학습 시간은 대조군 약 **23.01분**, 후보 **38.85분**이다. 기존 27개 진단 중 **23개 통과·4개 실패**이며, Town01 표지 오탐 25,548→16,702와 표지 IoU 6.44→6.93%는 개선했지만 recall 63.81→46.79%로 퇴행했다. 양쪽 전체 기록·최종 숫자 가중치·소스/입력/실행 제공 파일을 재검증하고 저장 예측 6개를 실제 정답 350장 전체 픽셀과 CPU에서 비교해 지표·실패 4개를 다시 계산했다. 이 학습은 현재 실행 중이 아니다. 이전 v2의 `BrokenPipeError`와 실패 원본은 보존하며 v3도 자동 교체·HD 지도 완성·독립 기능 승인으로 세지 않는다. [실제 결과와 한계](portable_e2e_learning_cycle_v1/22_map_native_fusion_pair/README.md).
- **제한 유지:** GPU0만, 개인 venv만, 다른 테스트와 충돌 금지, 실패 원본 보존, 차량 중심 화면, 설치·재부팅·실차 구동·자동 채택 없음. 코드 주석은 `HH_날짜 - English explanation`을 유지한다. 실제 크레딧 잔액·확정 토큰 비용은 조회하지 못한다.

<!-- HH_261002 - State the real low-speed curve data gap and preserve a rejected nominal route before any teacher capture. -->
<!-- HH_261002 - Add both actually completed matched controller attempts without rewriting the old learned-model failure. -->
**새 좌회전 제어 A/B도 실제 완료했다.** 같은 LEFT20k 모델로 추종 거리만 1.5m/0.8초 → 1.0m/0.5초로 줄였다. 같은 838개 모델 제어 상태의 중심 오차 p95는 **0.414→0.386m**, 약 6.8% 감소했지만 기존 0.35m에 미달해 **둘 다 FAIL**이다. 두 조건 모두 약 89° 회전·출구 뒤 10m·3초 이상 안정 주행, 차체·전체 가감속·100ms 기준은 통과했다. 각각 상태 1,055개·6카메라 528묶음·ACK 1,060개와 실패를 전량 보존하고 소유 서버·포트 정리도 확인했다. [읽기 쉬운 결과·남은 문제·다음 재학습](portable_e2e_learning_cycle_v1/24_actual_left_joint_lateral_lookahead_pair/OUTCOME.md), [전체 차량 중심 GIF 2개·PNG 10장](portable_e2e_learning_cycle_v1/24_actual_left_joint_lateral_lookahead_pair/README.md), [입력·소스 7,009개와 전체 미디어 재검증](../../../../storage/experiments/diagnostics/2026-10-02/temporal_left_joint_lateral_gallery_postcheck_v1/ROOT_actual_whole_record_postcheck.json). 반복성·독립 경로·정차·30km/h 또는 기능 승인을 뜻하지 않는다.

**이번 주행 개선의 이유:** 기존 회전 TRAIN 328개에는 미래 6.4초 전체가 저속 시험 범위인 샘플이 **0개**였다. 새 v2에서 실제 교사 좌회전 약 **89.9°·회전 후 복귀**를 재수집했고, 전체 차체·속도·명령 확인과 **347개 구간의 실제 미래 정답 22,208점** 검사를 통과했다. 학습 형식 변환의 JSON 자료 형식 비교 오류는 새 v3에서 수치·검사 기준을 바꾸지 않고 수정했다. **347개 전량 변환·서버 2,095개 파일 동일성·19개 TRAIN CPU 검사 후, 위의 실제 GPU 학습·전체 평가를 완료했다.** 실패한 v1과 v2 변환 시도는 보존하며 교사 성공이나 학습 중 손실을 모델 주행 성공으로 세지 않는다. [새 실제 데이터·전체 미래 검사](portable_e2e_learning_cycle_v1/18_low_speed_turn_data_readiness/README.md).

<!-- HH_261002 - Link genuine teacher pixels and retain the failed right-turn swept-body candidate without training it. -->
**새 실제 화면:** [좌회전 교사 전체 52.7초 GIF·6카메라·차량 중심 경로와 PNG 5장](portable_e2e_learning_cycle_v1/21_actual_low_speed_left_teacher_v2/README.md)을 만들고 원본·소스 3,421개와 미디어 6개를 전량 재검증했다. 이는 교사 자료이며 학습 후 모델의 주행 성공과 구분한다. [지도 손실 비교의 같은 영상·정답·모델 출력 10장](portable_e2e_learning_cycle_v1/19_map_sign_other_pair_loss/README.md)도 보존한다.

<!-- HH_261002 - Put the qualified fresh RIGHT697 before the preserved denied RIGHT70, without counting teacher completion as learned driving. -->
**새 우회전 80초 자료는 전량 변환·서버 검증 완료, 현재 새 균형 모델 학습에 사용 중:** 실제 교사 약 −89.97° 회전·출구 뒤 10m·3초 안정 주행과 차체·가감속·카메라·ACK 검사를 통과했다. 상태 1,755개·카메라 878묶음·ACK 1,760개 및 **697개 창의 실제 미래 44,608점**을 전량 검사해 실패 창 0을 확인했다. 별도 ROOT TRAIN 승인 뒤 697개 모두 학습 형식으로 변환했고, 서버 개인 `datasets/prepared`의 4,195개 파일 전량을 검증했다. 지도 GPU0 작업의 실제 종료 뒤 기존 직진·복귀·LEFT347과 균형 학습을 시작했다. 다음은 고정 최종 모델·전체 평가 → 실제 좌우회전과 직진 보호다. 교사 성공이나 변환 완료를 모델 기능 승인으로 세지 않는다. [새 자료·전체 원본](portable_e2e_learning_cycle_v1/18_low_speed_turn_data_readiness/README.md), [실제 새 학습](portable_e2e_learning_cycle_v1/26_actual_left_right_joint_learning/README.md).

**이전 우회전 70초 자료는 계속 학습 금지:** 상태 1,555개·카메라 778묶음·ACK 1,560개와 597개 창의 실제 미래 38,208점은 보존한다. 개별 미래 점 위반은 0이지만 출구 뒤 10m에서 3초 안정 주행 증거가 없어 전체 FAIL이다. 새 80초 성공으로 이전 실패를 바꾸거나 학습에 넣지 않는다. [전체 실패 판정](../../../../storage/experiments/validation/2026-10-02/town01_low_speed_right70_teacher_v2/pilot_00/diagnostic_report.json), [597개 전량 검사·학습 금지](../../../../storage/experiments/diagnostics/2026-10-02/town01_low_speed_right70_future64_v2/summary.json). 급한 경로의 기존 72개 실패도 그대로 보존했다.

**정리 완료:** 승인받은 pip 다운로드 캐시 약 1.2GB와 과거 `/tmp` 검사 폴더 4개 약 1.7GB를 정리했다. 설치된 패키지·venv·학습 데이터·실패 결과는 변경하지 않았다. 임시 폴더는 수정·미추적 소스와 Git 기록까지 서버 개인 폴더에 전량 보존했고 삭제 후에도 복구본을 검증했다. [삭제·복구본 영수증](../../../../storage/experiments/maintenance/2026-10-02/tmp_contract_snapshots_cleanup_v1/removal_result.json). 캐시는 다시 다운로드할 수 있고 임시 폴더는 영수증의 서버 백업에서 복구할 수 있다.

## 로컬 저장 공간이 늘어나는 이유

<!-- HH_261002 - Correct folder-name assumptions using actual full-tree file classes and deduplicated allocated blocks. -->
**19시 08분 파일 종류별 전수 조사: JPEG 원본이 약 70.30GB로 가장 크다.** 검증 폴더 32.52GB와 과거 `training` 폴더 37.78GB의 합계다. 실제 PT·PTH 체크포인트는 두 폴더 합계 약 2.99GB다. `training` 폴더 안에도 과거 CARLA 촬영 자료가 있으므로 52.48GB 전부를 가중치라고 해석하지 않는다. ROS `.db3` 7.79GB·녹화 `.mkv` 6.13GB 등도 공간을 쓴다. 디렉터리와 일반 파일·동일 inode를 구분한 [전체 종류별 수치와 삭제하지 않은 항목](../../../../storage/experiments/diagnostics/2026-10-02/local_capacity_file_classes_1908_v1/README.md)에 정리했다. 오늘 차량 중심 GIF·PNG 문서 폴더는 A/B 2개 추가 뒤 약 1.70GB, 준비 자료는 6.53GB다. 소스 코드 `tools`·`learning`은 합쳐 약 0.04GB다.

<!-- HH_261002 - Report read-only allocated storage, not an unmeasured before-after disk growth or permission to delete evidence. -->
앞선 17시 31분 읽기 전용 용량 조사에서 큰 폴더는 **주행 검증 원본 약 70.19GB**와 **training 폴더의 과거 수집·학습 결과 약 52.48GB**였다. 두 폴더는 당시 `storage/experiments` 약 123.04GB의 대부분이다. 다음 표는 **17시 31분 당시 오늘 파일**의 점유량이며 실행 전후의 순증가량을 측정한 것은 아니다. GB는 십진 단위다. 이후 좌회전 A/B 원본과 화면 추가 뒤 검증 전체는 약 71.15GB가 됐다.

| 10월 2일 파일 종류 | 점유량 | 용량이 생기는 이유 |
| --- | ---: | --- |
| 6카메라 JPEG 166,344장 | 16.51GB | 실제 수집 전체와 실패 꼬리를 각 시험에 보존 |
| 검증 시도별 모델·소스·기타 자산 | 3.18GB | 어떤 가중치·코드로 주행했는지 재현하기 위한 보존본 |
| 주행 상태·명령·시간 JSONL | 0.84GB | 전체 상태와 제어·지연 계측 기록 |
| 학습 결과 폴더 | 3.08GB | 체크포인트 1.66GB·수치 export 0.68GB 및 전체 평가 등 |
| 실제 GIF·PNG 문서 폴더 | 1.19GB | 차량 중심 전체 기록과 결과 화면 |

오늘 주행 검증 전체 22.43GB 중 카메라 이미지가 약 **74%**다. 새 우회전 자료 한 묶음도 원본 432MB·학습용 변환본 305MB·전량 미래 QA 64MB로 약 800MB다. 원본 5,268장 중 학습창에 해당하는 4,182장은 별도 복사돼 공간을 추가 사용한다. 저장소 코드가 주된 원인은 아니다.

대용량 과거 결과를 서버에서 전량 검증하고 로컬 보존본을 줄이면 공간 확보 효과가 크지만, 실패 원본·필요한 가중치를 임의로 지우지 않는다. 추가 승인된 임시 스냅샷 8개는 [서버 백업 검증](../../../../storage/experiments/maintenance/2026-10-02/tmp_contract_snapshots_cleanup_v2/backup_complete.json) 뒤 **실제 삭제·삭제 후 복구본 검증까지 완료**했다. 가용 공간은 직전 약 6.79GB에서 7.55GB로 늘었다. [정확한 8개 삭제 영수증](../../../../storage/experiments/maintenance/2026-10-02/tmp_contract_snapshots_remove_v4/removal_result.json)의 서버 개인 `archives/tmp_contract_snapshots_20261002_v2`에서 수정된 소스까지 복구할 수 있다. 첫 삭제의 **삭제 0개** [실패 영수증](../../../../storage/experiments/maintenance/2026-10-02/tmp_contract_snapshots_cleanup_v2/remove_failure.json)은 그대로 보존한다. 새 검사도 사용자·시스템 프로세스 접근 제한 221행을 정확히 고정했고, 실제 UID0·Kthread=1로 확인된 커널 worker 생성·종료만 구분했다. 모든 UID의 비사용을 입증했다고 주장하지 않는다. 전체 파일·백업·소유권·선언·프로세스 검증 및 삭제 직전 전체 바이트 재검사를 생략하지 않았으며 모델 안전 기준도 바꾸지 않았다.

<details>
<summary>오늘의 시각별 실행 상태·실패 원인·원본 검증 근거 펼치기</summary>

## 시각별 상세 근거

<!-- HH_261002 - Confirm actual private GPU0 CUDA forward/backward and optimizer rows rather than inferring training from an SSH process. -->
**10:14 KST 현재 — 새 주행 비교 학습이 실제 실행 중이다.** GPU0·기존 개인 venv에서 실제 TRAIN CUDA 순전파/역전파 검사를 통과했다. 원격 PID **3387727**, 실제 시작 **10:14:07 KST**, 대조군 **116/6,000 업데이트·464회 입력 노출**을 확인했다. 후보는 대조군 종료 후 같은 시작 가중치에서 6,000회 학습하며, 현재 후보 업데이트는 0이다. GPU 확인만 교정한 새 실행이고 원래 28개 학습 소스·데이터·계산·안전 기준은 그대로다. 새 실행 소스 29개와 별도 허가 SHA를 확인했다. [실제 실행·원본 로그](../../../../storage/experiments/training/2026-10-02/temporal_intrinsic_curve_pair_fit_dispatch_v2/started.json), [사전 고정 비교 조건](../../../../storage/experiments/plans/2026-10-02/temporal_intrinsic_curve_pair_v2/authorization.json)을 보존한다. 준비 검사나 SSH 시작만 학습으로 센 것이 아니다.

**다음 실제 성능 판정:** 두 최종 6,000회 모델 → 전체 71개 기존 진단 → 같은 경로·CPU4·PI2.4·0.5초 제어로 대조군/후보의 실제 주행 → 기존 Town07 직진·속도 보호 → 새 독립 경로 순서다. 반복 사용한 Town03 DEV와 신규 독립 평가를 구분한다. 새 실행 연결에서는 카메라 저장이 ACK 뒤에 이루어졌는지도 실제 기록으로 확인한다. 현재 **메인 0/9·세부 0/30**, 우선 기능은 직진·좌회전·우회전·종점 정지·속도 유지 5개이고 완료 5개가 아니다. 30km/h는 기본 성능 뒤, 60km/h는 후순위다. 자동 지도 모델은 별개로 유지한다.

**이번 실제 NN 회전 실패 화면도 게시했다:** [전체 110초 GIF·차량 중심 PNG 5장](portable_e2e_learning_cycle_v1/16_actual_town03_right/v3_verified_nn_controlled_green_basic_turn/README.md)은 실제 NN 효과 1,335개와 실패 후 긴급제동 꼬리를 포함한다. 마지막 6카메라/원경로/실제 궤적 화면을 직접 확인했다. 입력·소스 10,425개와 미디어 6개의 SHA 변화는 0개다. [전량 독립 검산](../../../../storage/experiments/diagnostics/2026-10-02/town03_known_dev_right_v3_data_admission_audit/independent_read_only_audit.json)에서 첫 이탈 제동은 70.75초·오차 1.009m이며, 차체 경계 위반은 그 전에 시작했다. 모델은 약 47°에서 멈춰 예정 약 91° 회전을 완료하지 못했다. 전체 카메라/ACK는 보존했지만 ACK 후 저장 로그는 없으므로 그 저장 순서의 측정 증거를 만들지 않는다. 이전 원본 FAIL을 덮거나 실패 데이터를 학습에 넣지 않는다.

이하 시각별 내용은 이전 실행·실패 기록이다. 현재 상태는 문서 맨 위의 최신 확인을 따른다.

<!-- HH_261002 - Distinguish the actual neural right-turn failure from preceding teacher-only wiring failures and the prepared next fit. -->
**09:59 KST 최신 확인 — 실제 모델 우회전은 아직 실패다.** 새 v3에서는 카메라와 모델 제어가 실제로 연결됐다. 추론 **668회**, 모델 제어 ACK **1,335회**, prefill **28회**를 기록했다. 회전 중 경로 이탈 한계를 넘어 `route cross-track limit exceeded` 안전 제동이 걸렸고, 회전 후 복귀를 완료하지 못했다. 중간 관측에서 실제 회전은 약 47°, 경로 오차는 약 1.07m였다. 실제 관측 묶음→ACK 최대 지연은 **90.12ms**로 기록돼 이번 정지를 그 구간의 100ms 시간 초과로 설명하지 않는다. 상태 **2,200개·6카메라 묶음 1,100개·ACK 2,204개**, 실패 이후 정지 구간까지 전체 110초를 보존했다. [원본 모델 제어 요약](../../../../storage/experiments/validation/2026-10-02/temporal_route_joint_town03_known_dev_right_v3/episode.partial/learned_summary.json), [실패 원본 판정](../../../../storage/experiments/validation/2026-10-02/temporal_route_joint_town03_known_dev_right_v3/diagnostic_report.json)을 확인할 수 있다. 판정 코드가 전체 카메라/ACK 완전성 검사에서 별도 오류를 냈으므로 그 원인과 차체·전량 제어를 독립 재검산 중이다. 실제 주행 실패와 분석 코드 오류를 서로 바꾸지 않는다.

**바로 이어갈 학습:** 같은 시작 모델·기존 정상 TRAIN·같은 순서에서 원래 손실 대조군과 회전 공간 경로 손실 보강 후보를 **각 6,000회** 비교한다. 새 head를 추가하지 않는다. PRO6000 개인 폴더의 기존 venv에서 TRAIN-only CPU 검증까지 완료했고 **아직 실제 optimizer 학습 시작 전**이다. GPU0는 방금 읽기 전용 조회에서 사용 프로세스 없는 1MiB·util 0% 상태였으며, 실행 직전 소유권과 CUDA 검사를 다시 한다. 두 최종 모델 → 전체 고정 평가 → 같은 로컬 주행 조건 → 새 독립 경로 순서다. 직진·속도 퇴행, 충돌·침범·차체 여유·처리 시간도 함께 유지해야 채택한다. 학습·평가는 앞선 실행 속도 기준 약 **1~2시간 예상**이며 확정 시간이 아니다. **메인 0/9·세부 0/30**은 그대로다.

<!-- HH_261002 - Keep a failed pre-optimizer launch separate from actual training and preserve its original traceback. -->
**10:02 KST 원격 실행 시도:** SSH 실행은 시작됐지만 기존 하위 GPU 검사에서 사용 프로세스 없는 1MiB 잔여 메모리를 거부해 **optimizer 업데이트 전에 종료**됐다. [실제 원본 오류](../../../../storage/experiments/training/2026-10-02/temporal_intrinsic_curve_pair_fit_dispatch_v1/actual_fit_stderr.txt)를 보존한다. 실제 학습 시작·완료로 세지 않는다. GPU0 UUID·빈 프로세스 목록·util 0%를 확인하고 작은 드라이버 잔여만 구분하는 새 실행 버전을 준비하며, 원래 GPU 소유권·단일 CUDA 장치·가중치·손실·평가 기준은 유지한다. 기존 소스·출력을 덮거나 다른 프로세스를 종료하지 않는다.

**이전 교사 주행 화면 게시 완료:** [v2 전체 약 93.4초 GIF와 차량 중심 PNG 5장](portable_e2e_learning_cycle_v1/16_actual_town03_right/v2_controlled_green_basic_turn/README.md)은 **교사만 주행한 모델 연결 실패**를 명시한다. 직접 두 실제 PNG를 확인했고 입력·소스 9,417개와 미디어 6개의 SHA 변화는 0개였다. 모델 궤적을 만들어 넣거나 교사 회전을 모델 성공으로 바꾸지 않는다. 이번 v3의 실제 NN 회전 실패 화면은 별도로 전량 원본을 확인한 뒤 게시한다.

이하 v1/v2와 이전 결과는 실패 분모를 보존한 과거 기록이다. 현재 실행 상태는 위 최신 확인을 따른다.

<!-- HH_261002 - Connect the current work to the user's nine-domain objective and report actual fit state without calling preparation training. -->
<!-- HH_261002 - Report the actual zero-inference v2 integration failure rather than relabeling a teacher drive or postprocessor error as neural performance. -->
**09:32 KST 현재 확인:** Town03 v2 기록은 끝났지만 **모델 추론·prefill·모델 ACK 모두 0회**다. 1,868개 실제 상태·934개 6카메라 묶음·1,872개 ACK가 있으며, 약 93.4초 동안 **교사로만** 주행했다. 모델 제어가 연결되지 않은 통합 실패이므로 우회전 모델의 성공으로 세지 않는다. 복제한 수집 함수가 신경망 연결 전의 원래 카메라·제어 함수를 보관한 원인을 확인했다. **새 v3에서는 실제 실행 시점의 두 연결을 적용하고, 모델 입력·제안이 전혀 없으면 출발 전에 거부하는 검사를 추가한다.** 기존 v1/v2 원본과 실패 판정은 보존한다. [v2 원본 판정](../../../../storage/experiments/validation/2026-10-02/temporal_route_joint_town03_known_dev_right_v2/diagnostic_report.json), [실제 추론 0회 기록](../../../../storage/experiments/validation/2026-10-02/temporal_route_joint_town03_known_dev_right_v2/episode/learned_summary.json)을 확인할 수 있다.

**모델 미실행 외의 실제 결과도 구분한다:** v2 교사 주행의 실제 가감속은 −1.913~+2.270m/s²로 기존 +3/−6 기준 안이었다. 반면 전체 1,868개 차체와 1,867개 중간 이동 검사에서 원래 차체 경계 위반은 90개·98개였다. 충돌·차선 센서 이벤트가 0이라고 차체 여유를 통과한 것은 아니다. 수동 1단 준비와 Green 신호 고정·원래 상태 복원은 원자료에 남았고, Green 조건은 신호 준수 기능의 증거가 아니다. 원래 위험 감지는 끄지 않았다. [독립 재계산](../../../../storage/experiments/diagnostics/2026-10-02/town03_teacher_only_integration_v2/root_read_only_audit.json)에서도 ACK·카메라 체인은 통과, 차체 경계는 미달이며 원자료 5,616개 파일 SHA는 그대로였다. 결과 분석 코드의 누락된 관측기 이름과 종점에서 호출되지 않는 교사 결정 1회를 잘못 요구한 오류도 함께 교정한다. **서버 새 주행 학습은 아직 시작 전**이며, 같은 시작 가중치에서 두 모델을 각 6,000회 학습하는 비교 코드를 검토 중이다. 신규 head가 아니라 회전 경로 손실을 보강하고 기존 직진·속도 성능을 함께 검사한다.

**이전 v1 실패:** Town03 우회전 시험을 실제 실행했지만 모델 제어 이전 출발 단계에서 중단됐다. 고정 0.11 페달·수동 1단에서 8초 안에 실제 속도 0.5m/s에 도달하지 못했다. 실제 상태 183개·6카메라 묶음 92개·ACK 187개를 모두 보존했고, 모델 추론·적용은 각각 0회다. **159개 실제 주행 상태 중 108개에서 원래 교사의 위험 감지 뒤 brake=0.5가 적용됐다.** 차량은 0.430m/s까지 움직인 뒤 제동됐으므로 페달 부족만으로 단정하지 않는다. 신호등과 장애물 중 어떤 감지였는지는 당시 원본에 기록되지 않아 확정하지 않는다. 우회전 모델이 실패했다거나 성공했다는 결과가 아니다. 이전 실패의 전체 110초 기록이 완료됐다고 주장하지 않는다. [실패 원본·소유권·전체 차체 검사](../../../../storage/experiments/validation/2026-10-02/temporal_route_joint_town03_known_dev_right_v1/diagnostic_report.json)를 보존한다.

<!-- HH_261002 - Retain the independently observed native braking violation and pre-neural initialization gap, not merely the watchdog error. -->
**독립 검산:** 159개 실제 속도 변화율에서 제동 순간 **−8.586m/s² 1회 미달**이다. 원자료 562개 파일은 SHA 변화 0개다. 카메라 시간·전체 183개 차체/182개 sweep은 통과했지만, 예외 정리 ACK의 1820 프레임이 마지막 관측 1819 밖이라 전체 ACK 검사도 미달이다. 관측하지 않은 프레임을 만들지 않는다. 준비 단계의 실제 제어는 자동 변속·기어 0였고 수동 1단은 주행부터 확인돼, 원래 준비 선언이 적용됐다는 주장도 하지 않는다. [전체 읽기 전용 검산](../../../../storage/experiments/diagnostics/2026-10-02/town03_known_dev_right_v1_launch_audit/root_read_only_audit.json)을 보존한다. 다음 새 버전은 위험 감지를 유지한 소유 서버의 명시적 기본 회전 신호 조건, 실제 제동 원인 기록, 준비 제어 확인을 고친다. 신호 준수 기능 성공은 별도다.

<!-- HH_261002 - Publish the complete actual pre-neural failed interval without inventing a model trajectory or Autoware screen. -->
**Town03 v1 실패 화면도 확인했다:** [전체 실제 약 9.15초 GIF·고정 PNG 5장](portable_e2e_learning_cycle_v1/16_actual_town03_right/v1_pre_nn_launch_failure/README.md)은 6카메라·차량 중심 경로·실제 주행 궤적을 포함한다. 마지막 화면을 직접 확인했고 모델이 실행되지 않았으므로 보라색 모델 계획은 없다. 원본·발행 입력 4,358개와 이미지/GIF 6개의 SHA는 바뀌지 않았다. 주행이 일찍 중단된 실패를 110초 시험 완료로 바꾸거나 Autoware/RViz 화면으로 부르지 않는다.

<!-- HH_261002 - Report the completed actual loss-only fit with both sign gains and recall/lane regressions; do not promote the candidate. -->
**새 지도 학습·평가 완료, 미채택:** 서버 GPU0·기존 개인 venv에서 **08:27:54~08:38:45 KST** 실행했다. 대조군/후보군 각 4,000회·32,000장 노출, 같은 원본·crop·순서·모델이며 손실만 다르다. Town01 배경→표지 오탐 픽셀은 **25,548→6,825**, 표지 IoU는 **6.44→12.37%**로 개선됐지만, 표지 recall은 **63.81→41.48%**, 차선 IoU는 **97.51→95.07%**로 퇴행했다. Town02 표지 IoU도 **75.32→80.34%**로 올랐지만 recall은 떨어졌다. 전체 350장·27개 판정에서 **21/27 통과·종합 FAIL**이므로 기존 모델을 교체하지 않는다. 전체 51개 원격 결과·로컬 미러의 SHA를 확인했고, 대조군은 앞선 두 대조군의 최종 가중치 SHA까지 재현했다. [전체 수치·실패 6개·가중치 근거](../../../../storage/experiments/training/2026-10-02/map_sign_margin_repair_v1/summary.json), [실행 선언](../../../../storage/experiments/plans/2026-10-02/map_sign_margin_repair_v1/authorization.json), [실제 업데이트 로그](../../../../storage/experiments/training/2026-10-02/map_sign_margin_repair_dispatch_v1/actual_fit_stdout.jsonl)을 보존한다. **이 새 지도 학습도 현재 실행 중인 것은 아니다.**

<!-- HH_261002 - Link the actually published fixed-view loss comparison without converting recognition gains into HD-map approval. -->
**새 지도 결과 화면:** [실제 RGB·정답·시작 모델·대조군·후보의 고정 PNG 10장과 전체 350장 수치](portable_e2e_learning_cycle_v1/15_map_sign_margin_loss_results/README.md)를 게시했다. 실제 Town01·Town02 화면을 직접 확인했고, 오탐 감소와 누락·차선 퇴행을 함께 표시한다. 신호 상태나 정밀지도 생성 성공으로 해석하지 않는다.

**이번 블록의 종료 기준:** 준비 파일 생성만으로 끝내지 않는다. 지도는 실제 고정 학습·전체 350장 평가·27개 기존/추가 기준의 결과, 주행은 실제 우회전의 진입·회전·복귀와 모든 실패 꼬리·제어/차체 검사·차량 중심 화면을 확인한다. 한 경로 성공을 메인 기능 완료나 모든 맵 성능으로 확대하지 않는다. 최초 예상은 약 30~60분·작업 토큰 1만~2만이며 출발 단계 실패로 추가 교정이 필요하다. 확정 청구량·잔액은 조회하지 못한다. 원격은 GPU0와 기존 개인 venv만, 로컬은 다른 시험과 충돌하지 않는 소유 서버만 사용한다.

<!-- HH_261002 - Replace the superseded running status with completed V2 results and the next causal diagnostic. -->
**추가 지도 학습과 평가 완료:** 06:57:54 KST에 PRO6000 GPU0·기존 개인 venv에서 시작한 두 모델의 4,000회 추가 학습이 모두 끝났다. 작은 표지 복습을 늘린 후보는 Town02 표지 IoU가 75.32→76.19%로 올랐지만, Town01 표지 recall이 63.81→51.42%로 떨어져 **15/20 통과·종합 FAIL, 미채택**이다. 현재 이 학습은 실행 중이 아니다. [비교표와 다음 교정](portable_e2e_learning_cycle_v1/10_map_sampler_second_results/README.md)에 정리했다.

<!-- HH_261002 - Record the completed effective-target census without claiming causal proof or continuing GPU activity. -->
**실제 학습 노출 전량 검사도 완료:** 서버 CPU에서 4개 학습군 전체 **128,000회**를 83.48초에 검사했다. 작은 표지의 crop·resize 후 실제 정답 픽셀은 **281,972→404,689, 43.52% 증가**했고, 표지가 완전히 잘린 비율은 5.70→5.56%였다. 추가 복습이 실제로 사라졌다는 가설은 맞지 않는다. 그렇다고 성능 하락 원인이 확정된 것도 아니다. [읽기 쉬운 검사 결과와 다음 손실 교정](portable_e2e_learning_cycle_v1/13_map_effective_training_exposure/README.md)에 전체 분모와 한계를 정리했다. 이 검사는 모델 추론·GPU·가중치 업데이트를 사용하지 않았다.

<!-- HH_261002 - Keep the fresh teacher capture visibly separate from neural training and any feature approval. -->
**v7 로컬 주행과 궤적 검사 완료:** 마지막 직선 구간의 정상 감속·정차 조향만 0으로 바꾸자 **교사 수집의 기존 16개 검사 모두 통과**했다. 3,564개 실제 상태·1,782개 6카메라 묶음·3,568개 ACK를 보존했다. 목표 약 1m 앞에서 속도 0.05m/s 이하의 2초 정차 유지와 6.5초 후속 기록을 완료했고, 정확한 속도 0도 마지막 8.05초 동안 확인했다. v6의 요청 조향·실제 적용값 불일치는 해결됐다. 하지만 전체 1,665개 창·106,560개 미래 점 검사에서는 **158개 궤적 창이 미달**이다. [성공/미해결 구분과 실제 전체 GIF·PNG](portable_e2e_learning_cycle_v1/14_actual_goal_stop_teacher_v7/OUTCOME.md)에 정리했다. v6의 149개보다 좋아졌다고 주장하지 않는다. 정상 가감속·충돌·침범과 별개인 저속 궤적 품질을 분리해 교정해야 하며, **v7도 학습 금지**다. 교사 정차 성공과 신경망 정차 성공은 다르다.

**주행 정차 원인 분석 완료:** 실제 1,665개 시점·106,560개 미래 점을 검사했다. 선택된 카메라 시점에서 종점 정보 전달은 거부 없이 통과했지만, 정지 직전 3개 시점은 준비된 디코더의 필요한 조건에 미달했다. 원자료의 149개 궤적 실패도 그대로 남아 **학습 금지**다. [정차 원인과 다음 순서](portable_e2e_learning_cycle_v1/12_stop_measurement_consistency/README.md)를 근거로 감속·정차 중 조향과 궤적 표현을 먼저 교정한다. 이것은 신경망 정차 성공 또는 기능 승인이 아니다.

**실제 지도 비교 화면 완료:** [같은 영상의 정답·기존 모델·일반 복습·두 후보 PNG 10개와 전체 수치](portable_e2e_learning_cycle_v1/11_map_sign_sampler_comparison/README.md)는 평가 전에 정한 고정 화면이다. 실패와 전체 350장의 분류 결과를 모두 보존했다.

<!-- HH_261002 - Keep the user's requested main9/sub30 status, completed training, next actual work, and operating constraints visible before detailed evidence. -->

- **학습 완료:** 서버 GPU0에서 주행 모델 6,000회 업데이트와 지도 모델 두 개 각 10,000회 업데이트·평가를 끝냈다. 현재 이 세 학습이 계속 진행 중인 것은 아니다.
- **실제 개선:** 같은 Town07 저속 조건의 중앙 출발에서 차로 중심 오차 p95가 0.224→0.035m로 줄었다. 다른 맵·30km/h·모든 기능의 성공은 아니다.
- **처리 시간 개선:** 같은 가중치의 CPU 4스레드 설정으로 좌·우·중앙 세 조건을 모두 다시 주행했다. 모두 기존 100ms 처리 기준을 통과했고, 시간 초과로 멈췄던 좌측 조건도 실제 복귀했다. 센서 노출부터의 전체 10Hz를 승인한 것은 아니다.
- **앞선 실패와 해결:** CPU4·0.3초 속도 추종은 전진 거리가 부족했고, 0.8초는 속도 상한을 넘었다. 같은 모델의 0.5초 비교에서는 세 조건 모두 거리·속도 기준을 만족했다. 두 설정의 여섯 실패도 보존한다.
- **진행 중 / 다음:** [0.5초 실제 3조건·0.3/0.8초 실패 비교·전체 GIF/PNG](portable_e2e_learning_cycle_v1/07_preview05_actual_signed45/README.md)까지 완료했다. 좌·우·중앙 세 조건 모두 기존 실제 기준과 독립 검산을 통과했다. 전진은 90.59·91.61·91.21m, 최대 속도는 모두 2.474m/s 미만이다. 수동 1단 무페달 계측도 완료했으며, 정상 출발·감속·목적지 정지 정답과 종점 표현을 확보한 뒤 혼합 재학습한다. 신호·ACC·회피 등 새 기능보다 기본 주행 성능을 먼저 확보한다.
- **지도 추가 학습·평가 완료:** 서버 GPU0에서 일반 복습과 오인식 배경 중심 복습을 각각 추가 4,000회 실행했다. 배경→표지 오인식은 25,548→22,311로 줄었지만 표지 recall은 63.81→59.66%로 퇴행해 **19/20 통과·종합 FAIL, 미채택**이다. [비교표·원본·다음 개선](portable_e2e_learning_cycle_v1/09_map_hardnegative_continuation_results/README.md)에 정리했다. 서버에서 이 학습이 계속 실행 중인 것은 아니며, 새 독립 평가 장소 0개라는 한계도 유지한다.
- **제한 유지:** GPU0만, 개인 venv 밖 설치 금지, 다른 시험 충돌 금지, 실패 원자료 보존, 실제 차량 중심 PNG/GIF, `HH_YYMMDD - English description` 주석. 크레딧 잔액·확정 토큰 비용은 조회할 수 없다.

<!-- HH_261002 - State the first five behavioral targets without counting preparation or a bounded pass as approved functionality. -->
**지금 집중하는 하위 목표는 5/30개:** 직진·좌회전·우회전·목적지 정차·속도 추종이다. 5개를 완료했다는 뜻은 아니다. 메인 영역 2·3의 기본 주행을 개선하면서 영역 9의 처리 시간·안전 동작도 함께 검증한다. 나머지 교통규칙·ACC·회피·차선변경·주차 등을 빼겠다는 뜻이 아니며, 기본 성능 확보 다음에 확장한다.

## 지금까지 개선된 점과 아직 안 되는 점

<details>
<summary>이전 학습·실제 주행의 전체 수치와 실패 기록 펼치기</summary>

<!-- HH_261002 - Link the complete recorded stop interval while preserving the failed teacher and future-quality verdicts. -->
**최신 정차 화면:** [실제 6카메라·차량 중심 경로·전체 178초 GIF와 PNG 5개](portable_e2e_learning_cycle_v1/08_actual_goal_stop_teacher_v6/README.md)를 만들고 정차·마지막 화면을 직접 확인했다. 교사 주행이며 신경망이나 Autoware/RViz 화면이 아니다. 실제 정지와 별개로 요청 일치·미래 궤적 검사가 실패한 사실도 화면에 표시했다. 실패 원자료를 학습에 넣지 않는다.

**이전 복귀 모델**은 Town07의 같은 저속 직진 경로에서 45초 시험 3회 모두 90m 이상 전진했다. 의도하지 않은 정지·충돌·차선 침범은 없었고, 차체 여유와 카메라 묶음 수신부터 명령 확인까지 100ms 기준을 만족했다. [원본 3회와 차량 중심 화면](../2026-10-01/temporal_route_recovery_fast45_results_v1/README.md)에 좋은 실행만이 아니라 전체 기록을 남겼다. 이 결과를 오늘 6,000회 공동 학습한 새 모델의 실제 성능으로 읽으면 안 된다.

하지만 약 8.6km/h 상한의 같은 경로 시험이며, 목적지 완주·종점 정차·좌우회전·30km/h 또는 모든 맵 성능은 아니다. 신호·ACC·회피·차선변경도 아직 모델 기능으로 입증되지 않았다. 센서 노출부터의 전체 벽시계 10Hz 또한 별도 확인이 필요하다.

PRO6000 GPU 0의 새 공동 학습은 최종 평가까지 완료했다. 이전에는 기존 97개 가중치를 동결하고 경로 복귀용 4개만 3,000회 학습했다. [이전 학습의 전체 결과](../2026-10-01/temporal_route_recovery_moving_v4_results_v1/README.md)는 48개 진단 중 20개 통과다. 이번에는 공유 부분·궤적·속도·복귀 가중치를 함께 업데이트하고 이전 직진·회전도 복습했다. [새 학습 결과](temporal_route_joint_results_v1/README.md)는 71개 진단 중 63개 통과·8개 미달이다. 검사 유형이 같아도 상대 개선 기준의 기준 모델이 바뀌므로 이전 20/48과 같은 절대 합격 점수로 비교하지 않는다.

학습에 넣지 않은 **같은 Town07 경로의 초기 조건 2개**에서 6.4초 미래 위치 평균 오차는 **0.641→0.046m**, 마지막 위치 오차는 **1.181→0.103m**, 속도 오차는 **0.213→0.021m/s**로 줄었다. 다른 맵·장소에 대한 독립 검증이나 실제 주행 결과를 뜻하지 않는다. 기존 자료의 곡률 2항목·속도 보존 6항목은 미달이며, 좋은 지표만 골라 모델을 배포하지 않는다.

<!-- HH_261002 - Distinguish an actual deadline-induced emergency latch from neural STOP behavior and retain the failed trial. -->
**새 공동 학습 모델의 첫 실제 시험은 실패했다.** 모델 제어 이후 전진은 0.446m이고, 45초 전체 원자료를 남겼다. 아홉 번째 추론에서 입력 처리·해시 34.793ms + 신경망 66.509ms + 나머지 2.445ms = **103.747ms**로 기존 100ms 제한을 넘었다. 안전 제동이 고정되어 이후 정지했으며, 모델의 STOP 제안은 0회였다. 정상 제어도 카메라 묶음 수신부터 명령 확인까지 최대 105.516ms로 미달이었다. 현재 프로세스 목록만으로 당시 다른 작업의 부하가 원인이었다고 단정하지 않는다.

같은 선언의 나머지 좌·우·중앙 비교를 보존해 재현을 확인한다. 이어서 **동일 입력·동일 가중치·동일 안전 기준에서 CPU 실행을 계측**해 처리 여유를 확보한다. 시간 제한을 늘리거나 실패 꼬리를 빼서 통과시키지 않는다. 오프라인 오차 개선과 실제로 잘 주행하는 성능은 별개다.

**CPU 비교도 실제 완료했다.** 같은 128개 입력에서 1·2·4스레드 전체 호출 p95는 85.37·67.76·62.37ms였다. 전체 출력의 사전 `1e-6` 비교는 디코더의 작은 float32 차이 때문에 실패했으며 원 결과를 보존한다. 4스레드의 제안 제어값은 128개 모두 그 범위 안이었다. [전체 결과와 한계](temporal_route_joint_cpu_threads_v1/README.md)를 근거로 별도 연구 설정의 실제 세 조건 시험도 완료했다. 모든 normal 제안 877회씩에서 실제 4/1 설정을 확인했고 세 조건 모두 기존 정책·관측 묶음→ACK의 100ms 기준을 통과했다. 좌측 조건은 9.35초에 복귀 밴드에 들어가 2초 유지했다. 엄격한 수치 동등성 실패를 성공으로 바꾸거나 시간 기준을 늘리지 않았다.

[CPU4 실제 3조건과 전체 45초 GIF·PNG](portable_e2e_learning_cycle_v1/05_thread4_actual_signed45/README.md)의 독립 검증에서 관측→ACK 최대는 76.74·77.17·77.87ms였다. 세 조건 모두 차체 여유·속도 상한·처리 시간을 통과했지만 전진 거리 기준은 미달이다. 실제 6카메라·차량 중심 지도·원 경로·지나온 궤적·이미 적용된 모델 계획을 한 화면에서 볼 수 있다. Autoware/RViz 화면 촬영과는 구분한다.

**목적지 정지 교사 자료 1회도 실행했지만 아직 학습에 넣지 않는다.** 실제로 210.05m를 전진해 목표 0.544m 앞에서 속도 0으로 2초 유지하고 6.5초 미래 문맥까지 기록했다. 그러나 정지 직전 실제 감속이 -16.32m/s²까지 커져 품질 검사에 실패했다. 2,092개 상태·1,046개 카메라 묶음·실패 원자료를 보존한다. 교사의 목표 정지 완료와 학습 가능한 편안한 정지 정답, 모델의 자율 정지 성능은 서로 다르다. 급제동이 나온 제어 전환을 먼저 바로잡는다.

**정지 교정 v2도 실제 실행했지만 실패했다.** 정상 brake 상한 0.10→0.01, 감속 목표 1.5→0.2m/s²를 함께 바꿔 먼저 감속하는 비교였다. 목표 0.974m 앞에서 2초 정지했고 2,277개 상태·1,139개 6카메라 묶음·2,282개 ACK·차체 여유·전체 소유권은 확인했다. 그러나 저속에서 가속·제동을 반복하며 실제 최대 감속은 -21.19m/s²로 오히려 나빠졌다. 브레이크 값을 낮추는 것만으로 해결되지 않는다는 결과다. 전체 실패를 보존하고 학습하지 않으며, 차량 응답과 제어 전환을 먼저 분리해서 진단한다. 정지 완료를 좋은 학습 자료 또는 모델 정지 성공으로 세지 않는다.

[v2 전체 위반 10구간과 원인 구분](town07_natural_goal_stop_teacher_v2/README.md)에서는 실제 float32 brake 0.01과 가속·제동 반복, actor world 속도 급락을 확인했다. 조기 full brake나 카메라 표시 문제만으로 설명되지 않으며 내부 물리 원인은 아직 확정하지 않았다. 실제 미래 1,022개 창·65,408개 점 전량을 검사했고 실패 181개 창과 종점 근처 짧은 GO 12개를 그대로 보존했다.

**정답 자료와 현재 궤적 디코더의 불일치도 찾았다.** [실제 미래 64점 전수 진단](town07_natural_goal_stop_teacher_v1/FUTURE64_FINDINGS.md)은 929개 기준 시점·59,456개 실제 미래 점을 검사했다. 그중 종점 근처 GO 라벨 65개는 남은 경로가 18m 미만인데 현재 GO 디코더는 18m를 요구한다. 단순히 학습 횟수를 늘려도 이 표현 불일치는 해결되지 않는다. 실패 구간을 삭제하거나 GO를 임의로 BRAKE로 바꾸지 않고, 실제 종점 거리와 정지 궤적을 표현할 수 있게 수정한 뒤 기존 직진·회전과 함께 재학습해야 한다. 현재 실패 자료에는 학습 권한을 부여하지 않았다.

[무엇을 어떻게 연결해서 고칠지](../../../portable-e2e-terminal-aware-repair-plan-2026-10-02.md)에 현재 코드의 미사용 의도·정지 거리·가속도 출력, 잘린 경로와 실제 목적지의 구분, 새 종점 계약, 정상 정답 → 수치 표현 검증 → 혼합 학습 → 실제 주행 순서를 정리했다. 종점 정보 계약과 새 수치 디코더 코드는 준비됐지만, 실제 정답 전량의 표현 가능성 검사·모델 학습·실행기 연결·정지 기능 승인은 아직 완료하지 않았다. [계속 지킬 사용자 조건](../../../../AGENTS.md)도 저장소 작업 지침에 남겼다.

### 실제로 개선된 부분과 이번 실패를 한눈에 보기

<!-- HH_261002 - Show matched actual native measurements rather than substituting offline loss or approval counters for driving performance. -->

**아래 표는 초기 비교의 실패 기록이다.** 중앙 출발의 실제 차로 중심 오차는 크게 줄었지만, 좌측 조건은 처리 지연으로 멈췄고 나머지는 전진 거리 기준에 미달해 당시 전체 0/3이었다. 이후 CPU4·0.5초 제어 설정의 실제 세 조건은 모두 통과했으며, 그 최신 결과는 다음 절에 구분했다.

| 실제 조건 / 지표 | 이전 모델 | 새 공동 학습 모델 | 판단 |
| --- | ---: | ---: | --- |
| 중앙 출발: 차로 중심 오차 p95 | 0.224m | **0.035m** | 약 84% 감소. 같은 Town07 저속 조건의 실제 개선. |
| 중앙 출발: 순간 속도 최대 | 2.516m/s | **2.472m/s** | 기존 2.5m/s 초과 → 상한 내. |
| 중앙 출발: 전진 거리 | 96.16m | 88.71m | 기존 90m 기준 미달. 출발·가속과 목표 속도 표현을 더 개선해야 함. |
| 우측으로 벗어난 출발: 복귀 | 8.05초 진입 + 2초 유지 | 8.85초 진입 + 2초 유지 | 복귀 통과지만 더 빨라진 것은 아님. |
| 우측으로 벗어난 출발: 전진 거리 | 96.18m | 89.36m | 90m 기준 미달. |
| 좌측으로 벗어난 출발 | 96.17m / 복귀 통과 | 0.45m / 시간 초과 안전 정지 | 실행 시간 문제를 먼저 해결. |

위 복귀는 밴드 진입 후 2초 유지 조건이다. 이후 모든 순간 계속 중앙에 있었다는 의미는 아니다. [전체 3조건·실패 꼬리·실제 차량 중심 화면](temporal_route_joint_signed45_results_v1/README.md)과 원본 상태를 보존한다.

</details>

<!-- HH_261002 - End the detailed-history fold before showing every requested main capability. -->
</details>

## 메인 9개 영역에서 지금 하는 일

| 메인 영역 | 하위 수 | 현재 근거와 남은 일 |
| --- | ---: | --- |
| 센서·상태 이해 | 4 | 실제 6카메라·차량 상태 배선은 있음. 차선·도로 이해, 실제 센서 보정과 결함 대응은 미완료. |
| 경로·차로 주행 | 4 | **현재 핵심 작업.** 직진의 한정 반복 성공 다음으로 좌우 복귀·좌우회전·목적지 정차를 개선하고 독립 경로에서 검증. |
| 속도·승차감 | 3 | **현재 핵심 작업.** 궤적과 속도를 함께 개선하고 정상 가감속을 검증. 기본 주행 이후 30km/h, 60km/h는 후순위. |
| 교통규칙·교차로 | 4 | 전문가 신호 대기 기록은 모델 기능이 아님. 신호·정지선·우선권·보행자 라벨과 상호작용 학습 필요. |
| 선행차 추종·ACC | 3 | 선행차·간격·끼어들기 장면과 모델 학습 필요. |
| 충돌위험·장애물 대응 | 4 | 안전 제동과 모델의 회피 능력을 구분. 장애물·합법 통과·양보 데이터와 독립 AEB 검증 필요. |
| 차선변경·합류 | 3 | 좌우 변경·후방 간격·합법성·중단과 복귀 장면 필요. |
| 저속 정차·주차 | 2 | 기본 경로의 종점 정지와 별개로 갓길·후진·근거리 데이터 필요. |
| 런타임 안전·실차 전환 | 3 | 한정 시험의 시간·제어 소유권 검사 완료. 전체 10Hz·fault/fallback·실차 비구동 검증은 미완료. |

## 이어갈 작업과 종료 기준

<!-- HH_261002 - Replace the superseded 6000-step running plan with the actual balanced fit and its fixed evaluation-to-driving sequence. -->
1. **현재 주행 학습:** PRO6000 GPU0에서 정상 LEFT347·RIGHT697과 기존 직진·복귀를 함께 고정 **20,000회·100,000개 노출**로 학습 중이다. 실제 19시 38분에는 4,638회를 확인했다. 같은 101개 가중치 구조이며 추가 head는 없다. 10시의 두 6,000회 비교와 뒤의 좌회전 전용 20,000회 학습·실제 A/B는 이미 끝났고 실패도 보존했다. 현재 학습은 처음 처리 속도 기준 약 2~3시간과 추가 평가가 예상된다.
2. **최종 평가 다음 실제 주행:** 고정 마지막 모델의 전체 업데이트·입력 순서·101개 숫자 가중치·소스 동일성을 확인한 뒤 부모와 최종 모델을 같은 기존 74개 기준에 비교한다. 기존 실패 14개를 유지하고 RIGHT 3개를 더한 전체 77개 오프라인 진단을 공개한다. 그다음 새 export 실행기를 별도로 검토하고 로컬에서 같은 조건 좌우회전·직진 보호 → 새로운 독립 경로 순으로 실제 주행한다. 동일한 안전·차체·지연·전체 실패 보존 기준을 유지하며, 정차 실패 자료와 Town03 DEV는 학습에 넣지 않는다. 한 경로·오프라인 통과를 메인 기능 완료로 세지 않는다.
3. **지도 개선:** 구조 비교 v3도 양쪽 20,000회와 고정 350장 전수 평가를 마쳤고 **23/27·실패 4개로 미채택**이다. 원본 52개와 모든 저장 예측을 다시 검산한 [동일한 고정 PNG 10장](portable_e2e_learning_cycle_v1/25_map_native_fusion_v3_results/README.md)을 게시했고, 전체 원본·소스와 10장 전체 픽셀도 재확인했다. 표지 오탐 감소와 동시에 재현율 저하가 남았으므로 가중치를 자동 교체하지 않는다. 이후 누락 원인을 분리한 새 비교와 신규 독립 장소 평가를 거쳐야 하며, 실시간 입력·주행 기록의 좌표 누적·차선 연결은 별도 남은 지도 단계다.
4. **나머지 기능:** 기본 주행·속도·정차 이후 교통규칙 → ACC → 장애물 → 차선변경·합류 → 저속·주차를 확장한다. 온라인/기록 기반 자동 지도는 별도로 개발하며 부정확한 지도를 주행 모델에 강제로 넣지 않는다.

<details>
<summary>10월 2일 전체 실험 순서·원본 링크·세부 검증 수치 펼치기</summary>

<!-- HH_261002 - Distinguish the current actual local control experiment from completed remote training and formal main capability approval. -->
**05:15 KST 진행 상황:** 동일 최종 6,000회 가중치·CPU4·PI2.4를 유지하고 속도 추종 시점만 바꾼 비교다. 0.3초는 거리 미달, 0.8초는 속도 상한 초과로 각각 0/3이었다. 중간값 0.5초는 실제 좌·우·중앙 세 조건 모두 기존 거리·속도·복귀·차체·처리 시간 기준을 통과했고, 원본 전량 독립 검증과 실제 GIF/PNG도 완료했다. 새 모델 학습이나 30km/h 시험이 아니라, 현재 메인 영역 2·3·9의 기본 성능을 막던 제어 설정을 비교한 결과다. 모든 조건에서 충돌·침범 0, 차체/10cm 여유, 기존 처리 100ms, 추가 실제 변화율 +3/−6m/s²도 통과했다.

<!-- HH_261002 - Separate the first successful physical precursor from the still-running declared batch and formal capability approval. -->
**중간값 0.5초의 실제 세 조건은 모두 통과했다.** 같은 사전 선언·설정·기준을 유지했다. 첫 한 번을 선택한 결과가 아니며, 가중치를 다시 학습한 변화가 아니라 동일 모델의 속도 추종 시간 설정 비교다. 아래 원본 진단 수치는 완료된 독립 검산에서도 확인했다.

| 출발 조건 | 0.5초 전진 거리 | 실제 속도 최대 | 차로 중심 오차 p95 | 전체 실제 변화율 범위 | 기존 기준 |
| --- | ---: | ---: | ---: | ---: | --- |
| 좌측 이탈 | 90.591m | 2.473834m/s | 0.239m | −1.983~+2.339m/s² | PASS |
| 우측 이탈 | 91.614m | 2.473368m/s | 0.228m | −2.027~+2.290m/s² | PASS |
| 중앙 | 91.206m | 2.473281m/s | 0.039m | −1.981~+2.474m/s² | PASS |

**원자료 독립 검산도 통과했다.** [0.3·0.5·0.8초 전체 9시도 비교](../../../../storage/experiments/validation/2026-10-02/temporal_route_joint_preview05_signed45_batch_audit_v1/README.md)는 0.5초의 전체 2,700상태·1,350카메라 묶음·2,712ACK와 제어·실제 차체·속도·시간을 다시 계산했다. 세 조건의 관측 묶음 수신→ACK 최대는 82.410·76.018·79.609ms다. 좌·우 이탈 복귀는 각각 8.30·8.40초 밴드 진입 후 2초 유지했으며, 중앙 조건을 세 번째 이탈 복귀로 세지 않는다. 0.3·0.8초의 여섯 실패도 보존했다.

[차량 중심 실제 화면](portable_e2e_learning_cycle_v1/07_preview05_actual_signed45/README.md)은 세 조건마다 전체 45초 GIF와 5개 PNG, 총 18개다. root가 실제 마지막 6카메라·경로·주행·이전 적용 모델 궤적을 직접 확인했다. 이미지 생성 후 추가 근거 index를 같은 이름으로 중복 생성하려던 후처리 오류는 [별도 완료·오류 기록](portable_e2e_learning_cycle_v1/07_preview05_actual_signed45/PUBLICATION_COMPLETION.json)에 보존했다. 기존 화면·원본·독립 결과를 덮거나 재실행하지 않았고, 실제 주행/독립 검산의 실패와 구분한다.

이 표는 약 9km/h 이하 Town07 직진의 제한된 전단계 결과다. 정식 기능 승인 **0/9·0/30**을 바꾸지 않으며, 회전·목적지 정지·30km/h·다른 맵·실차 성능은 별도로 검증한다.

**수동 1단 무페달 45초 계측도 완료했다.** [전체 실제 상태·명령과 원인 구분](town07_manual_first_coast_v1/README.md)에서 1.528→0.021m/s, 전진 10.960m, 준비부터 끝까지 급감속 기준 위반 0회를 확인했다. 목적지 정지 교사나 모델 자율 정지 시험은 아니며 학습 권한도 없다. 최초 2초 저속 유지 후 작은 재가속이 있었으므로, 무페달만으로 영구 정지한다고 해석하지 않는다.

<!-- HH_261002 - Keep prepared terminal code separate from actual teacher capture, model training and deployment. -->
**현재 주행 작업은 목적지 정차다.** 정상 주행 중 브레이크를 반복하지 않고, 실제 목표 거리에서 코스팅한 뒤 목표 앞 1m·실제 속도 0.05m/s 이하에서만 정지 유지 제동을 허용한다. 기존 위험 상황 긴급 제동은 유지한다. 교사 자료가 성공해도 모델의 자율 정지 성공으로 세지 않는다.

<!-- HH_261002 - Distinguish a per-episode quota interruption from a physical stop failure and preserve every attempted version. -->
**v4 실제 시험은 1GiB 기록 한도로 중단됐다.** 208.237m까지 진행한 3,092개 실제 상태의 급감속 기준 위반·충돌·침범은 0회였으나, 목표 정차와 후속 기록을 완료하지 못했다. 마지막 목표 잔여 거리는 2.361m, 실제 속도는 0.235m/s였다. 전체 디스크 부족이나 모델 정차 실패로 단정하지 않으며, 미완료 자료를 학습에 넣지 않는다. 원자료와 실패 꼬리는 보존했다.

<!-- HH_261002 - Record the actual v5 protocol failure and keep the fresh v6 ordering intervention separate from teacher/model capability. -->
**v5 실제 수집·전수 검사는 완료했지만 학습 미승인이다.** 목표 약 0.956m 앞에서 실제 속도 0.033m/s로 2초 저속 유지하고 6.5초 후속 기록까지 확보했다. 준비부터 끝까지 가감속·충돌·침범·차체 여유는 통과했으나, 정차 유지 40회 모두 교사가 제안한 brake=1이 두 번째 일반 제동 상한 처리로 brake=0이 되어 실제 적용됐다. 목표 저속 유지가 관측됐다는 것과 선언한 정차 제어가 적용됐다는 것은 다르다. 후속 실제 미래 검사도 1,665개 창을 모두 보존했고, 169개 창에서 품질 미달을 확인했다. 정차 유지 이전 코스팅의 저속 궤적 문제도 있어 제어 순서 수정만으로 학습 품질까지 보장하지 않는다.

**v6 실제 시험·전수 검사 완료:** 원래 두 제동 상한 처리 뒤 정차 유지 명령을 적용하도록 순서를 고쳤다. 정차 제동 40회 모두 실제 ACK brake=1이고, 목표 약 1m 앞에서 실제 속도 0으로 2초 유지했다. 전체 가감속·충돌·침범·차체·카메라·소유권은 통과했다. 그러나 기존 정차 조향 억제가 요청 steer 약 0.005를 0으로 바꿔, 엄격한 요청 일치 검사는 여전히 실패다. 모든 3,410개 정상 명령에서 원래 조향 억제 함수를 재적용해 이 한 필드 외 불일치 0회를 확인했다. 기존 판정을 덮지 않는다.

실제 64점 미래도 1,665개 창·106,560개 점 전량을 검사했고 **149개 창 미달**이다. 저속 코스팅의 6개 실제 프레임쌍과 최초 정차 제동의 1개 프레임쌍이 겹친 미래 창에 반복 포함된다. 441개 곡률 위반 점을 441회 독립 사고로 세지 않으며, 정의되지 않은 4,941개 곡률은 0 정답으로 만들지 않는다. v6도 학습 미승인이다. 다음은 같은 물리 시험을 무작정 반복하는 것이 아니라, 실측 위치·속도·정지 거리의 일관성과 새 종점 디코더의 표현 가능성을 먼저 검산한다. 이는 메인 영역 2·3의 목적지 정차 준비이며 모델 자율 정차 성공이 아니다.

실제 전체 목적지와 잘린 앞쪽 경로를 구분하는 [종점 정보 계약 코드](../../../../learning/portable_e2e/terminal_witness_v1_20261002.py)와 [새 종점 수치 디코더](../../../../learning/portable_e2e/path_time_decoder_terminal_v1_20261002.py)를 준비했다. 일반 주행은 기존 디코더에 그대로 위임하고, 종점에서만 실제 현재 속도와 검증된 경로 끝을 사용한다. 경로를 멋대로 늘리거나 실측 속도를 0으로 바꾸지 않으며 실제 모델 입력·학습·실행기에는 아직 연결하지 않았다. 방금 root가 통과한 v5/v4 교사·지도 준비 검사 209개는 코드 검사이지 물리 주행이나 기능 승인 수가 아니다.

이어갈 순서는 **정상 정차 원자료 확보 → 실제 64점 미래와 정지 곡률 정의 검사 → 목적지 인식 궤적 표현 연결 → 기존 직진·회전과 섞은 GPU0 학습 → 같은 조건 실제 주행과 독립 경로 비교 → 30km/h 확대**다. 정상 정차 자료가 불합격이면 실패를 보존하고 원인을 고친다. 학습 횟수만 늘려 표현 불일치를 덮지 않는다.

**현재 지도 작업은 표지 오인식과 누락의 동시 개선이다.** 승인 TRAIN 1,209개만 전량 계측하고 같은 10,000회 시작 모델에서 두 후속 비교를 완료했다. 구조·손실·고정 영상 crop은 같고 복습 비율만 달랐다. [첫 후속 결과](portable_e2e_learning_cycle_v1/09_map_hardnegative_continuation_results/README.md)에서는 배경 오인식이 약 12.7% 감소했지만 recall이 4.151%p 하락해 미채택이었다. 이어 작은 표지 복습을 늘린 두 번째 비교도 끝났으며 Town01 recall이 더 떨어져 미채택이다. [동일 영상과 모든 분류 비교](portable_e2e_learning_cycle_v1/11_map_sign_sampler_comparison/README.md)에 두 후보의 실패를 함께 정리했다. 두 번째 결과 47개 파일의 원격·로컬 SHA 일치를 확인했다. 현재 추가 학습은 종료 상태이며 다음은 crop에 실제 표지 픽셀이 남는지 확인하는 CPU 검사다. DEV·holdout을 optimizer에 넣지 않고, 신규 독립 지도 평가 장소 0개와 자동 배포 금지를 유지한다. 7분류 인식은 전체 정밀지도 생성 완료가 아니다.

| 출발 조건 | 0.3초 → 0.8초 전진 거리 | 실제 속도 최대 | 복귀 밴드 진입 | 전체 기준 |
| --- | ---: | ---: | ---: | --- |
| 좌측 이탈 | 87.94→92.93m | 2.516m/s | 9.35→7.10초 + 2초 유지 | 속도 상한 초과 FAIL |
| 우측 이탈 | 89.36→93.55m | 2.514m/s | 8.80→7.85초 + 2초 유지 | 속도 상한 초과 FAIL |
| 중앙 | 88.77→93.27m | 2.515m/s | 원래 중앙 조건, 복귀 횟수에 더하지 않음 | 속도 상한 초과 FAIL |

위 수치는 실제 전체 native 기록을 보존한 owner 진단과 독립 감사에서 확인했다. [0.8초 전체 3조건·45초 GIF·PNG·검증 근거](06_preview08_actual_signed45/README.md)에 3개 GIF·15개 PNG와 실패를 모두 남겼다. 0.8초 변경은 제어 설정 개선/실패이지 새 가중치 학습 성능이라고 부르지 않는다.

<!-- HH_261002 - Report an actually started GPU0 job and keep its pre-fit evaluation distinct from completed optimizer updates. -->
**실제 실행 시작:** 10월 2일 01:02:28 KST에 PRO6000 물리 GPU 0에서 다음 공동 학습 작업을 시작했다.
학습 전 전체 비교값 계산 → 고정 6,000회 업데이트 → 최종 전체 평가까지 완료했다. 실제 GPU 소유 시간은 약 45분 49초이고, 학습 전 평가를 포함한 학습 단계는 약 37분 3초, 최종 평가는 약 8분 46초였다.
동일한 101개 가중치 텐서 구조에서 54개 텐서, 약 228만 개 숫자 파라미터를 학습하고 영상 입력부·의미 head 등 47개 텐서를 고정한다.
TRAIN 18개 주행 기록·새 미학습 조건 2개를 구분하며, 24,000회 노출을 독립 주행 수로 세지 않는다.
새 속도 손실과 이전 회전·직진 복습을 함께 사용한다. STOP·30km/h는 이번 학습 완료만으로 승인하지 않는다.

지도는 새 p02·p04 실제 480쌍 수집·전수 QA·고정 원본/semantic 겹침 검토까지 완료했다.
[지도 비교 학습 계획](map_paired_balanced_v1/TRAIN_PLAN.md)은 주행 GPU 작업 종료 후 직렬 실행한다.
[수동 검토 화면](map_paired_appearance_review_v1/README.md)은 실제 원본·정답이며 모델 예측 화면이 아니다.
지도 모델의 현재 학습 범위는 RGB로 도로·차선 등 7개 종류를 분류하는 영상 인식이다. 깊이와 실제 위치는 정답·평가에만 사용하고 모델 입력으로 넣지 않는다. 새 480쌍은 기존 TRAIN 지역의 추가 장면이며, 새 독립 DEV 장소는 0개다. 신호 상태·차선 연결 관계·좌표 누적까지 갖춘 정밀지도 자동 생성 완료를 뜻하지 않는다.
주행 작업 종료와 GPU 비어 있음을 확인한 뒤 지도 비교 학습을 실제 시작했다. 각 모델 10,000회·80,000회 자료 노출, 총 2개 모델을 순서대로 학습한다. 같은 원본 729개와 새 물리 장면 480개를 사용하며, 첫 모델은 OFF만, 두 번째는 같은 장면의 OFF/ON을 번갈아 본다. 두 최종 가중치를 저장한 뒤에만 기존 평가 자료를 열어 비교한다.

**지도 비교 학습·전수 평가도 완료했다.** 실제 학습 시간은 대조군 10분 59.7초·균형군 10분 54.7초다. [전 분류 수치와 원본/정답/두 모델 예측 PNG](map_paired_balanced_results_v1/README.md)에 20,000개 학습 기록과 기존 평가 350장 전체를 검산한 결과를 정리했다. 균형 학습은 대조군보다 도로·차선 등 대부분 지표가 낮아 광범위 개선이라고 채택하지 않는다. Town01 표지는 IoU 약 6%대로 여전히 크게 부족하며, 도로 픽셀이 많아 전체 정확도가 높아도 표지·신호등 성능을 대신하지 않는다. 독립 신규 지도 지역과 실제 좌표·차선 연결·랜드마크 누적은 다음 지도 단계다.

[동일 평가 350장의 이전 모델·대조군·균형군 비교](map_paired_balanced_results_v1/PARENT_COMPARISON.md)에서는 대조군의 Town01 차선 IoU가 이전 모델보다 약 1.66%p, 신호등 약 30.88%p 개선했다. 작은 대상인 표지는 여전히 약 6%대여서 충분한 지도 품질로 승인하지 않는다.

1. **완료:** 공동 학습 최종 6,000회 평가·101개 텐서의 숫자 검증·12개 중간 기록 보존. **진행:** 새 가중치의 실제 폐루프 시험. 새 복귀 자료와 기존 직진·회전의 위치·속도 성능을 모두 비교하며, 작은 경로 분기만 반복 학습하는 것으로 전체 향상을 가정하지 않는다. 이번 새 복귀 자료에는 정상 출발·종점 정차 정답이 없으므로 STOP 학습 완료를 주장하지 않는다.
   **초기 실제 3조건 비교도 완료:** 중앙 오차·속도 상한은 개선했으나 실행 지연·전진 거리 미달로 전체 0/3이었다. **후속 CPU4 실제 3조건도 완료:** 처리 시간과 좌우 복귀는 통과했고 전진 거리 미달은 남았다. **후속 0.5초 실제 세 조건·독립 검산·화면도 완료:** 거리와 속도까지 통과했다. 다음은 정상 감속·종점 정지 자료와 궤적 표현의 연결이다. 단순 증량 학습보다 현재 확인된 표현 불일치를 먼저 해결한다.
2. 최종 가중치를 실제 CARLA에 연결해 복귀·더 긴 경로·회전을 확인한다. 시험별 준비·안전 조건을 지키며, 짧은 시험 하나가 끝났다고 메인 목표 달성으로 종료하지 않는다.
3. 독립 장소·날씨·seed와 전체 성공률을 늘린다. 직진과 좌우회전은 각각 최소 20회 등 기존 로드맵 조건을 그대로 적용하며, 좋은 실행만 골라내거나 본 결과에 맞춰 기준을 낮추지 않는다.
4. 지도는 별도 모델로 OFF/ON 영상 조건에 맞춘 자료 확보와 차선 누락 개선을 이어간다. 주행 모델에 부정확한 지도를 강제로 섞지 않는다. 같은 GPU 0이나 CARLA를 사용하는 작업은 직렬 실행한다.
5. 기본 주행·속도·정차 확보 후 교통규칙 → ACC → 장애물 대응 → 차선변경·합류 → 저속·주차로 넓힌다. 메인 9개 목표는 계속 추적하며, 기능마다 head 하나를 붙이는 식으로 구현하지 않는다.

새 패키지 설치·실차 구동·다른 작업 종료 등 추가 권한이 필요하거나 자원이 다른 테스트에 예약되어 있으면 구체적인 상황을 보고한다. 사용자가 새로 요청하지 않아도 완료된 학습 다음의 평가·안전한 후속 작업을 이어간다. 실행 중인지, 완료인지, 준비 중인지를 구분해 보고한다.

### 로컬 복귀 시험의 실행 오류도 남긴다

<!-- HH_261002 - Preserve a pre-capture configuration failure without treating it as a driving-model failure or overwriting its declaration. -->
새 좌·우 복귀 시험 `v1/trial_00`은 CARLA 실행 후 수집을 시작하기 전에 실패했다. 난수 번호 `20261002100`이 기존 NumPy의 최대값 `2**32-1`보다 컸다. 실제 차량 상태·카메라·모델 제어 기록은 0개이며, 이것은 모델의 복귀 성능 평가 결과가 아니다. 원래 선언·로그·실패 결과를 유지하고, 허용 범위를 사전에 검사하는 새 버전에서 다시 선언한다. 같은 오류가 확정된 나머지 두 v1 조건은 재실행하지 않는다. 소유 프로세스 종료·TCP/UDP 포트 해제는 정상 확인했다.
`v2`는 활성화 전 검증 연결 문제로 사전 검사에서 실패했고 CARLA를 실행하지 않았다. 수정한 `v3`에서는 좌·우·중앙 3조건을 실제 45초씩 수집했다. 좌측·우측은 모델 제어로 각각 6.35초·8.05초에 복귀 범위에 들어와 2초 유지했다. 그러나 세 조건 모두 실제 순간 속도가 2.5m/s 상한을 약간 넘어 **전체 기준은 0/3**이다. 이를 속도 값을 잘라 통과시키거나 짧은 복귀 성공으로 덮지 않는다. 새 공동 학습 가중치는 같은 설정으로 별도 비교한다.

</details>

## 두 환경과 계속 지킬 제한

로컬은 CARLA 자료 수집·모델 추론·실제 폐루프 시험·차량 중심 PNG/GIF를 담당한다. PRO6000은 개인 `hwanhong` 폴더와 기존 `venvs/py312`에서 물리 GPU 0만 사용해 학습·평가한다. GPU 1, 공용 환경, 다른 사용자 프로세스·시뮬레이터는 건드리지 않는다. 임의 설치·재부팅·자료 삭제·자동 배포는 하지 않는다. 변경 코드에는 `HH_YYMMDD - English description` 주석을 남긴다.

10월 2일 04:56 KST 확인에서 원격 가용 공간은 약 3.12TB, GPU0는 사용 0MiB·여유 97,250MiB였다. 04:49 KST 로컬 가용 공간은 약 22.04GB였다. 완료된 학습을 계속 실행 중이라고 보고하지 않는다. 이것은 확인 시점의 값이지 자원 예약이나 학습 시작 기록은 아니며 실제 실행 직전에 다시 확인한다. 크레딧 잔액과 확정 토큰 비용은 조회할 수 없다.
