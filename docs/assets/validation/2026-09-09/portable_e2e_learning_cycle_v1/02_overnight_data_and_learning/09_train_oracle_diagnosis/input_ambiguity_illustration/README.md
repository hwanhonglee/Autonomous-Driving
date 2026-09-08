# 같은 입력, 다른 미래 정답 — 원본 사례 설명

<!-- HH_260906 - Preserve original expert images and labels; this is a post-hoc explanatory view, not a driving test. -->

전체 11쌍 중 **첫 번째 쌍을 사후에 선택한 설명 자료**입니다. Town01 sequence 294와 434는 약 14초 차이지만, 원래 카메라 JPEG 6장과 모델 입력 해시·현재 위치·자세·경로가 같습니다. 미래 정답은 정지 유지와 출발로 다릅니다.

![동일한 원본 카메라 6장](same_six_camera_inputs.png)

![원본 미래 경로와 속도](different_future_labels.png)

두 PNG는 원래 전문가 데이터로 만든 오프라인 설명 그림입니다. **학습 모델의 실제 주행·Autoware 화면·새 녹화가 아닙니다.** 경로 그림의 검은 삼각형은 원점의 차량 **위치 표시이며 차량 방향을 나타내지 않습니다.** 신호 상태 정답은 기록되지 않아 신호 변화가 원인이라고 단정할 수 없습니다.

현재 입력만으로 결정하는 모델은 두 가지 다른 미래 정답을 모두 정확히 맞출 수 없습니다. 이것이 정답 오류나 모든 오차의 유일한 원인이라는 뜻은 아닙니다. 데이터 승인·제어 승인·모델 승격·새 학습은 수행하지 않았고, 정답·시간·표본을 수정하거나 삭제하지 않았습니다.

- [두 원본 샘플 JSONL](original_pair_samples.jsonl): 정확히 두 줄, 원본 바이트 유지.
- [원본 실행 보고서](report.json): 6회 TRAIN 진단의 증거 SHA와 두 사례의 원본 이미지 경로 연결.
- [공개 출처 기록](provenance.json): 12개 입력 SHA, 원본/공개 파일 매핑과 변환 구분.
- [실행 소스의 공개용 사본](illustrate_identical_training_inputs.public.py): 개인 작업공간 경로만 placeholder로 바꾼 비실행 사본. 원래 실행 소스 SHA는 보존합니다.
- [원본 체크섬 기록](original_SHA256SUMS.txt), [이 폴더 체크섬](SHA256SUMS).

원본 JPEG 6장은 `original_images/`에 있으며, 두 시점의 바이트가 같으므로 한 벌만 보관합니다. 부모 폴더의 전체 11쌍·6,882개 분석 결과를 대체하지 않습니다.
