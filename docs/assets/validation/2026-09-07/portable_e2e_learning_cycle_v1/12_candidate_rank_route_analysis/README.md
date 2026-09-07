# 후보 경로 인지 점수 모델 E · 3개 seed 전체 경로 분석

<!-- HH_260906 - Publish measured ego-centered validation predictions without claiming learned driving or promotion. -->

이 폴더는 **학습된 E 모델의 검증 데이터 예측 분석**입니다. 새 CARLA 주행 촬영이나 Autoware 주행 화면, GIF가 아닙니다.
각 seed의 GPU 학습 1,540 step·val337 평가·geometry v8 감사가 모두 끝나고 동일 checkpoint SHA가 확인된 뒤에만 CPU 진단을 실행했습니다.
진단 대상은 동일 v3 corpus의 Town03 우회전 val337입니다. 학습은 train1,147개이며 Town04 held-out test는 열지 않았습니다.

## 전체 검증 337개 수치

| Seed | GPU selected ADE m | CPU selected ADE m | CPU−GPU m | CPU oracle ADE m | CPU regret m | ADE oracle 일치 | 선택 c0–c5 | 후보 간 평균 경로 거리 m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: |
| 20260903 | 4.73572502 | 4.73572969 | +0.00000467 | 1.63474321 | 3.10098648 | 213/337 | 0/0/131/65/141/0 | 7.644209 |
| 20260904 | 4.53910535 | 4.53910351 | -0.00000184 | 1.52799451 | 3.01110888 | 190/337 | 261/0/0/76/0/0 | 7.553578 |
| 20260905 | 4.22841463 | 4.22841787 | +0.00000324 | 1.62355125 | 2.60486674 | 210/337 | 139/174/24/0/0/0 | 7.551834 |

CPU와 GPU의 계산 결과를 별도로 보존했습니다. 위 작은 수치 차이는 원본 값을 대체하지 않습니다.
CPU 진단은 selected/oracle ADE·선택 빈도·후보 간 거리를 산출합니다. FDE·속도 MAE의 CPU 재평가나 GPU 성능 측정은 하지 않았습니다.
정확한 signed ADE 차이와 oracle ADE 차이, 실행 전후 source 확인은 [CPU/GPU 비교 JSON](cpu_gpu_comparison.json)에 있습니다.

## 고정 위치 경로 PNG 18장

그림은 예측을 보기 전에 고정한 `0, 67, 134, 201, 268, 336`을 사용했습니다. `floor(phase × 336 / 5)`로 val 전체 진행 구간을 균일하게 선택했습니다.
모든 그림에서 차량은 중앙입니다. 회색=route, 초록색=expert, 색상=6개 모델 후보, 굵은 색상=선택 후보입니다. 각 그림은 모든 점이 들어가도록 축척을 정하므로 그림끼리 픽셀 길이를 직접 비교하지 마세요.

- **Seed 20260903** — [0](seed_20260903/val_phase_000.png), [67](seed_20260903/val_phase_067.png), [134](seed_20260903/val_phase_134.png), [201](seed_20260903/val_phase_201.png), [268](seed_20260903/val_phase_268.png), [336](seed_20260903/val_phase_336.png); [val337 전체·sample별 JSON](seed_20260903/selector_diagnostic.json)
- **Seed 20260904** — [0](seed_20260904/val_phase_000.png), [67](seed_20260904/val_phase_067.png), [134](seed_20260904/val_phase_134.png), [201](seed_20260904/val_phase_201.png), [268](seed_20260904/val_phase_268.png), [336](seed_20260904/val_phase_336.png); [val337 전체·sample별 JSON](seed_20260904/selector_diagnostic.json)
- **Seed 20260905** — [0](seed_20260905/val_phase_000.png), [67](seed_20260905/val_phase_067.png), [134](seed_20260905/val_phase_134.png), [201](seed_20260905/val_phase_201.png), [268](seed_20260905/val_phase_268.png), [336](seed_20260905/val_phase_336.png); [val337 전체·sample별 JSON](seed_20260905/selector_diagnostic.json)

## 해석과 제한

첫 seed 20260903의 E는 C보다 GPU ADE가 악화됐지만 FDE는 개선됐습니다. 선택하는 후보 종류가 늘어도 선택 품질 개선을 보장하지 않습니다.
ADE oracle은 유효 expert XY에 대해 ADE가 가장 작은 후보이며, speed 등을 포함하는 학습 loss oracle과 다릅니다. Regret는 selected ADE−oracle ADE입니다.
후보 간 경로 거리는 15개 후보 쌍×64개 예측 시점의 평균 Euclidean 거리입니다. 후보가 서로 다르다는 진단이지 충돌 회피나 주행 안전성의 증거가 아닙니다.
C/E는 동일 corpus·seed·optimizer step을 사용하지만 점수 모델의 구조와 파라미터 수가 다릅니다. 성능 변화 원인을 궤적 정보 하나만으로 단정하지 않습니다.
이 결과로 운용 모델을 교체하거나 실제 차량 제어를 승인하지 않습니다. 전체 상대·절대 품질 판정은 3-seed C/E 캠페인 요약을 따릅니다.

## 실행 근거와 원본 검증

- 학습·진단 source: `5c50353480a51f11b970c7d0b9ec8dcd91f7b056`
- 진단 script SHA-256: `1524e18a92758a8dc95bf86a4d7c11524a139e5c027fd4e6d9ba0f17a7dfc022`
- CPU wrapper SHA-256: `2ab070c87080c44e9cc0b95a327bf6a922c260023c7e1a4c24e9ea0215d9c68e`
- 고정 캠페인 plan SHA-256: `43dc9c69e35b5c47776bffe36c936d4bfa40ddedc3a9c4c5525e3ba3b7af5a34`
- 개인 venv만 사용하고 CUDA_VISIBLE_DEVICES를 비웠습니다. OMP/MKL 및 torch intra/inter-op은 각각 4 threads였습니다.
- 각 진단 전후 git HEAD·tracked/untracked clean 상태·script SHA·최종 GPU 보고서/체크포인트 SHA를 재확인했습니다. 패키지 설치·GPU 사용·학습 소스 수정은 하지 않았습니다.
- 실행 wrapper와 계정별 경로가 있는 원본 provenance는 private artifacts에 보관하고, 공개 JSON에는 해당 원본 SHA와 경로 없는 실행 확인만 기록했습니다.

| Seed | 최종 checkpoint SHA-256 | CPU 진단 소요 초 |
| --- | --- | ---: |
| 20260903 | `b3b77502b7c5e600d5d78a09cee1c3d8e9afdb168d8a78a241d39abce273b5e0` | 11.529 |
| 20260904 | `41b6f89d2d2cbcc05f4f5f79ad0e799e00c97ec86884d7c3f47fd2fac548ab8c` | 11.524 |
| 20260905 | `bbd14a0c27d36fa7c90927b8c84176d709a1bd21df068ea89703af410f836188` | 11.525 |

소요 시간은 val 자료 확인·CPU 추론·PNG 저장을 포함합니다. live 10 Hz 충족이나 GPU inference latency를 뜻하지 않습니다.
원본 JSON 3개와 PNG 18개를 bytes 그대로 발행했습니다. JSON 내 PNG hash 및 [SHA256SUMS](SHA256SUMS)로 검증할 수 있습니다.
