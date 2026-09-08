# nuPlan 카메라 calibration: 정적 opcode 목록

<!-- HH_260906 - Static metadata inventory is not trusted calibration, deserialization, dataset admission or model training. -->

기존 계획과 별도로 사전 고정한 후속 검사입니다. 같은 DB 멤버 1개를 메모리에서 압축 해제하고 전체 SHA를 확인했습니다. DB 파일을 디스크에 풀지 않았으며 이미지·LiDAR·지도 payload는 읽지 않았습니다.

카메라 8개 × 필드 4개 = **32개 BLOB**(5,320 bytes)을 모두 검사했습니다. 인코딩 파싱 거부는 0개입니다. `GLOBAL`·`REDUCE` 등은 이름/위치만 기록했으며 함수나 생성자를 실행하지 않았습니다.

| 필드 | BLOB 수 | bytes 합계 | 인코딩 파싱 | 실행 의미 미해결 opcode 수 |
|---|---:|---:|---:|---:|
| translation | 8 | 1688 | 8/8 | 72 |
| rotation | 8 | 1816 | 8/8 | 80 |
| intrinsic | 8 | 1328 | 8/8 | 16 |
| distortion | 8 | 488 | 8/8 | 0 |

**이 결과는 안전하게 해석된 숫자 calibration이 아닙니다.** opcode의 숫자·문자 인자는 인코딩 관찰일 뿐 K/TF/회전·왜곡 값으로 검증되지 않았습니다. 스택·참조·순환 구조도 미해결이며 모든 필드는 `UNRESOLVED_INVENTORY_ONLY`입니다. 모델 학습·변환·데이터 이용약관 승인·차량 적용은 하지 않았습니다.

32개 원본 `.bin`은 비공개 진단 폴더에만 보존하며 여기에는 복사하지 않습니다. 공개 JSON은 기계별 경로를 치환할 수 있는 metadata view이고 원본 및 공개 SHA를 각각 보존합니다. 원본 전체 ZIP은 이번에 다시 해시하지 않았습니다: ZIP stat 일치와 과거 ZIP 해시를 기록하고, 이번 DB 멤버 전체 및 32 BLOB 해시는 직접 확인했습니다.

- [전체 opcode 관찰](report.json)
- [사전 고정 후속 계획](followup_plan.json)
- [필드별 집계](opcode_summary.json)
- [실행된 원본 검사 코드](execution_source.py)
- [비공개 원본 파일의 해시 목록](original_SHA256SUMS.txt)
- [원본→공개 SHA 및 재검증 기록](publication_manifest.json)

수집·공개 helper는 로컬 비공개 BLOB을 같은 정적 parser로 다시 확인했습니다(DB 재열기 없음). 재실행에는 비공개 원본 36개 파일, 이전 report, 고정된 검사 소스와 별도의 새 출력 폴더가 필요합니다. `git clone`만으로 원본 데이터나 이용 권한이 생기지는 않습니다.

원본 실행: `2026-09-08T19:19:36.528635+00:00` → `2026-09-08T19:19:37.274851+00:00`; 상태 `OPCODE_INVENTORY_COMPLETE_NOT_DECODED`.
