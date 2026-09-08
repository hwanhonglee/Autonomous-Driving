# nuPlan 보정값 literal 진단 — 숫자 형식만 확인

<!-- HH_260906 - Publish numeric sanity without claiming a verified coordinate basis, projection, rig, or training admission. -->

8개 카메라·32개 필드의 저장된 숫자 슬롯을 검사했고, 유한값·K 구조·주점 범위·quaternion norm 검사만 8/8 통과했습니다. **보정값 신뢰·TF 적용·실제 센서 배치·이미지 투영·학습 사용은 모두 미승인입니다.**

이번은 2026-09-09 04:41:10 KST(2026-09-08 19:41:10 UTC)에 별도로 선언한 로컬 진단입니다. 앞선 opcode-only 계획의 권한을 소급 확대하지 않았습니다. 기존에 보존된32개 BLOB에 대해 전체 비숫자 템플릿을 맞춘 뒤 지정된 IEEE-754 슬롯만 읽었습니다. pickle VM·생성자·NumPy·ORM은 실행하지 않았고 DB·ZIP·이미지를 다시 읽지 않았습니다.

| 카메라 | metadata 해상도 | fx / fy | cx / cy | quaternion norm 오차 | 숫자 sanity |
| --- | --- | ---: | ---: | ---: | --- |
| CAM_B0 | 1920×1080 | 1545 / 1545 | 960 / 560 | 2.22e-16 | PASS · 미신뢰 |
| CAM_F0 | 1920×1080 | 1545 / 1545 | 960 / 560 | 1.11e-16 | PASS · 미신뢰 |
| CAM_L0 | 1920×1080 | 1545 / 1545 | 960 / 560 | 0 | PASS · 미신뢰 |
| CAM_L1 | 1920×1080 | 1545 / 1545 | 960 / 560 | 1.11e-16 | PASS · 미신뢰 |
| CAM_L2 | 1920×1080 | 1545 / 1545 | 960 / 560 | 1.11e-16 | PASS · 미신뢰 |
| CAM_R0 | 1920×1080 | 1545 / 1545 | 960 / 560 | 0 | PASS · 미신뢰 |
| CAM_R1 | 1920×1080 | 1545 / 1545 | 960 / 560 | 0 | PASS · 미신뢰 |
| CAM_R2 | 1920×1080 | 1545 / 1545 | 960 / 560 | 1.11e-16 | PASS · 미신뢰 |

8개에 공통인 K literal: `[1545.0, 0.0, 960.0, 0.0, 1545.0, 560.0, 0.0, 0.0, 1.0]` (모두 동일: True).
공통 distortion literal: `[-0.356123, 0.172545, -0.00213, 0.000464, -0.05231]` (모두 동일: True).

이 값이 같은 것은 저장 값의 관측일 뿐, 실차가 동일한 광학계라는 증거가 아닙니다. distortion 계수 순서·모델, optical 축과 변환 방향, 카메라와 차량 좌표계 관계, 실제 해상도·왜곡·투영 정확성은 별도 검증해야 합니다. quaternion을 보정하거나 값·시각을 바꾸지 않았습니다.

[별도 계획](plan.json) · [전체 진단과32개 필드](report.json) · [8개 숫자 요약](camera_sanity.json) · [실행 source](execution_source.py) · [원본/공개SHA](publication_manifest.json) · [공개 체크섬](SHA256SUMS)

[이전 opcode 진단](../calibration_opcode_inventory/README.md) · [실제 데이터 준비 상태](../README.md)

## 참고 소스와 남은 제한

- [nuplan/database/common/data_types.py](https://raw.githubusercontent.com/motional/nuplan-devkit/e9241677997dd86bfc0bcd44817ab04fe631405b/nuplan/database/common/data_types.py) — commit `e9241677997dd86bfc0bcd44817ab04fe631405b`, SHA256 `8151180b93c99dda0ea1d9ec5d39644342783efc2a42fd1b3df47e5a9b8f381d`.
- [nuplan/database/nuplan_db_orm/camera.py](https://raw.githubusercontent.com/motional/nuplan-devkit/e9241677997dd86bfc0bcd44817ab04fe631405b/nuplan/database/nuplan_db_orm/camera.py) — commit `e9241677997dd86bfc0bcd44817ab04fe631405b`, SHA256 `f18eb1bc36d6b3c0743f177b8fb529e708956640e3444580376554e063efe0b8`.

공식 참고 코드 두 파일은 검증에 사용한 private 사본으로만 보존했습니다. 여기에는 링크·commit·SHA만 싣고 원문 코드를 재배포하지 않습니다. 원본32개 BLOB·DB·이미지는 공개하지 않습니다. 원본SHA 파일은 비공개6개 원본용 참조이며, 이 폴더는 자체SHA256SUMS로 검증하세요.

다음에는 이미지 대응과 투영 검증, 8개 중 실제 사용할6개 rig 정의, native 시간 정렬, 변환 방향·축 검증, 약관 검토가 필요합니다. 이번 진단은 해당 검증이나 데이터 변환·학습·모델 추론을 수행한 것이 아닙니다.
