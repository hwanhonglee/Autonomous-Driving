# VisionPilot 도입 전 환경·센서·출력 사전 검사

<!-- HH_260906 - Separate completed local prerequisite discovery from model execution, benchmark readiness and installation authorization. -->

**2026-09-08 18:23:22 KST**, 로컬 Python 3.10.12에서 사전 검사기를 실행했습니다.
검사 종료 코드는 0이고 `preflight_status=COMPLETE`입니다. 모델 실행은 `NOT_RUN`,
도입 준비 상태는 `NOT_READY`입니다. “검사 완료”와 “모델 실행 가능”은 같은 뜻이 아닙니다.

## 실제 발견한 환경

| 항목 | 관찰 | 아직 알 수 없는 것 |
|---|---|---|
| ONNX Runtime Python 모듈 | 이 interpreter 검색 경로에서 미발견 | 다른 venv·private SDK 존재 여부 |
| `cv2` | 모듈 위치 검색에서 발견 | import·실제 영상 처리·ABI 호환성 |
| `onnx` | 모듈 발견, 배포 metadata 1.18.0 | 이것은 추론 엔진이 아니므로 모델 실행 가능 여부 |
| 시스템 동적 라이브러리 캐시 | OpenCV core 4.5 계열 발견, ONNX Runtime SONAME 미발견 | C++ 헤더·private SDK·실행 시 로딩 가능 여부 |

`find_spec`·배포 metadata·고정 `ldconfig -p` 명령만 사용했습니다. ONNX Runtime·OpenCV·Torch·
CUDA 모델 import, GPU 조회, 모델 가중치 다운로드, 패키지 설치, 빌드, SSH 접속은 하지 않았습니다.
모듈 발견은 10 Hz 추론이나 차량 제어 가능성의 증거가 아닙니다.

## 현재 설정을 연결하려면 남은 일

<!-- HH_260906 - Distinguish manually declared current configuration from measured live sensor properties. -->

아래 현재 값은 CLI로 명시한 **설정 선언**이며 실시간 센서 측정값이 아닙니다.
저장소의 센서 YAML 원본 해시도 함께 남겼지만 검사기가 YAML을 파싱해 선언과 대조한 것은 아닙니다.
전방 640×360·70°·sensor_tick 0.1초와 카메라 6대는 원본 설정을 별도로 읽어 확인했습니다.
실제 camera Hz·전송 지연·보정 품질은 이 검사에서 측정하지 않았습니다.

| 차이 | 필요한 작업 |
|---|---|
| 6카메라 입력 대 단일 전방 입력 | 전방 영상 선택과 전처리 정의 |
| 전방 640×360·70° 대 공식 약 1–2MP·50–55° 기준 | 적합한 별도 센서 profile·intrinsic·extrinsic 검증. 단순 확대가 원래 세부 정보를 복구하지는 않음 |
| Portable 시간별 XY·속도 궤적 대 VisionPilot 조향·가속 출력 | 좌표·단위·타임스탬프·출력 의미를 맞추는 비제어 어댑터 설계 |
| 목적지 조건부 도심 기능 대 차로 내 L2 기능 | 먼저 차로 유지·완만한 곡선·앞차 추종 같은 공통 과제에 비교 범위 제한 |
| CARLA 0.9.15 대 공식 안내 0.9.16 | 브리지 호환성 확인. 자동 업그레이드하지 않음 |

공식 센서 수치는 조사한 기준이며 모든 지원 해상도의 확정 목록이 아닙니다. 예를 들어
통상 2MP로 부르는 1920×1080이 정확한 2,000,000화소보다 크다는 이유만으로 실행 불가라고
단정하지 않습니다. 검사기의 보수적 검토 표시는 별도 profile 적합성 확인이 필요하다는 뜻입니다.

공식 코드 버전·근거와 전체 비교 순서는 [Portable·VAD·VisionPilot 비교 계획](../../../../../portable-e2e-visionpilot-comparison.md)에 있습니다.
현재 공식 가중치 무결성·추론·전처리 일치·10 Hz 지연·출력 권한 분리는 모두 미검증입니다.

## 재실행 방법

<!-- HH_260906 - Offer a standard-library-only diagnostic without adding system, Conda or native dependencies. -->

저장소 루트에서 실행합니다. Python 표준 라이브러리만 사용하므로 이 검사를 위해 새로 설치할
패키지는 없습니다. 아래 출력 폴더가 이미 있으면 다른 새 이름을 사용하며 기존 결과를 지우지 않습니다.

```bash
PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES='' python3 -m scripts.e2e.visionpilot_preflight \
  --output-dir artifacts/visionpilot_preflight_local_01 \
  --sensor-config autoware_e2e_vad_launch/config/sensor_mapping_portable_e2e_10hz.yaml \
  --camera-count 6 --front-width 640 --front-height 360 --front-hfov-deg 70 \
  --camera-hz 10 --output-kind timed_trajectory \
  --task route_conditioned_urban --carla-version 0.9.15
```

[실제 실행 명령](execution.json), [선언한 입력](report/declared_inputs.json),
[환경 관찰·남은 검증 전체](report/report.json), [생성 파일 SHA](report/SHA256SUMS)를 보존합니다.
설정 파일·스크립트가 실행 중 바뀌거나 출력이 이미 있거나 데이터셋 경로이면 거절합니다.
중간에 파일 생성이 실패하면 완성 manifest가 없으므로 보고서에 COMPLETE 문자열이 있더라도
완료 증거로 인정하지 않습니다. source 커밋은 `990085c`이며 전용 단위 검사 49개가 통과했습니다.
이 49개는 이후 [최종 전체 코드 검사](../25_stop_primitive_research/verification.json)의 일반
테스트 4,422개에도 포함됐습니다. 모델 추론을 49번 실행했다는 뜻은 아닙니다.

**원격 Pro6000은 이번 검사에서 접근하거나 변경하지 않았습니다.** 앞으로 사용하더라도
GPU 0만, 패키지는 hwanhong 개인 venv 안에서만 허용된다는 조건을 유지합니다.
공식 C++ 스택의 시스템 라이브러리 설치는 venv 패키지 설치와 다르므로 별도 범위 확인 없이
`apt`, `.deb`, Conda base, CUDA/드라이버 변경을 실행하지 않습니다.
