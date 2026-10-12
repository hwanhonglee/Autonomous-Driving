# Town05 어두운 도로 지도 투영: 실행 전 고정 계약

<!-- HH_260929 - Freeze a repeated-DEV three-frame hard-case replay before model inference. -->

목적은 기존 AI v2 RGB 모델이 Town01 배선에서 보였던 모습과 달리 **Town05_Opt의 어두운 노면/회색 차선**에서 어떤 지도 증거를 만드는지 보여주는 것이다. **탐색용 실패 진단**이며, 이미 반복 사용한 Town05 DEV를 새 독립 DEV 또는 TEST라 부르지 않는다. 결과를 보고 threshold·seed·프레임·모델을 바꿔 최선의 경우를 고르지 않는다.

- 원본: `storage/experiments/training/2026-09-27/map_town05_independent_dev_capture_epic_v2/mapping/`. 이 경로의 `training` 디렉터리명은 실제 역할을 뜻하지 않는다. summary SHA-256 `99dcd8a68b36f0f24611c512f2471139f9f155ea222b697ce899090239376c89`; records SHA-256 `966058d688e901469e1199070392a5c1c97d7e93041c992125a9c18d5c0a7f6a`; 수집 감사 SHA-256 `d64d77bf4f1d02892f59ac9debd18489c5acbf38842ec81ac4398f7052b950d1`. route SHA-256 `efb8ccca533f25cae54b2eb7e03274c15795211488128185334ff2bc55f4744c`, 240 원본 프레임, 물리 BasicAgent, RGB/depth/semantic/pose 동기 수집, 충돌 0·live/replay 일치.
- 원본 순서에서 **0000/0115/0231**을 고정한다. RGB PNG SHA-256은 순서대로 `6fb392d75fb0b8f330b1c41fbac780c1cb539089ad8b28ebdc09e5f88dc39258`, `a6322a4f11e8f5bc1fceabe4c89bb7f81a61574790b1d5ac6ed2ed1c7f286447`, `3a66683bae948bea92cbcaff6f60f7ae90a733f0372f970f5bea965f953eac85`. NPZ SHA-256은 `0eb681ae5deef4c189c369fe2362b7a4b3e8124dc6420b33df052504b17bc8d9`, `b5bc2dfbcb63ad9a154e0877545ca647fdcf9164849ef43136fefb5ef7a75bcf`, `915d2741bdcdd19240ec2016ca1576d969ab64f16cb007184ae49b15bef338f0`이다.
- 모델은 이전 [Town01 배선 실험](../map_learned_projection_bridge/PLAN.md)과 같은 AI v2 checkpoint SHA-256 `ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380`, 소스·도구·임계값을 사용한다. RGB 640×360만 모델에 입력한다. stride3, oracle 광축 깊이 0.1–40m, oracle pose, confidence ≥0.5, `other` 제외, 0.2m 격자다. CARLA semantic은 오직 진단 정답/기준 지도에만 사용한다.
- CPU 2스레드만 사용한다. GPU·CARLA·Autoware·SSH·설치·새 영상 수집·새 가중치 학습 없음. 단 3장만 건너뛰어 재생하며 그 사이 **237장은 누적하지 않는다**. 새 출력 폴더 `run_v1`을 만들고 원본 입력·예측 마스크/확률·서로 다른 출처의 격자 JSON·차량 중앙 전체시야 PNG·per-frame 7클래스 혼동행렬·해시를 남긴다. 성공은 재현 가능한 배선/오류 가시화뿐이며, map quality/10Hz/HD-map 승격은 없다.

사전 중단 조건: 입력·모델·소스 해시 불일치, Town/route 또는 센서 pose/시간 계약 불일치, 출력 디렉터리 이미 존재, 원본 품질 감사 불일치. 이 경우 이전 결과를 덮어쓰거나 다른 프레임을 고르지 않는다.
