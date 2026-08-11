# PC2 v1.1 verification report

Test window: 2026-08-10 through 2026-08-11 KST.

## Verdict

| Scope | Result |
|---|---|
| PC2 `run_autoware` launch integration | PASS |
| Lucid serial-214 live image and CameraInfo | PASS |
| generic YOLOX inference and bounded ROI contract | PASS |
| permanent YOLO-to-fusion graph wiring | PASS |
| TLR component/model initialization | PASS |
| final TLR semantic output | NOT PROVEN — PC3 map/dynamic TF absent in final standalone run |
| useful camera-LiDAR fused 3D output | NOT PROVEN — PC3 LiDAR absent in final standalone run |
| local PC2 no-actuation boundary | PASS for observed process/topic graph |
| guarded shutdown and cleanup | PASS for final cycle |
| sustained final 3-PC five-cycle acceptance | NOT COMPLETED |
| road-driving readiness | NOT ESTABLISHED |

## Focused build and test evidence

### CenterPoint TensorRT wrapper

- changed resource order to context, engine, plan, runtime;
- focused package build succeeded;
- the recorded 86-test clean result predates the final source edit and is retained only as a
  baseline; it is not claimed as a post-fix unit-test result;
- isolated two-engine launch loaded the voxel encoder and head engines;
- SIGINT exit code 0 with zero `runtime.cpp`, `mEngineCounter`, or deserialized-engine errors.

### YOLOX post-processing

- focused package build succeeded;
- post-fix package artifacts report 19 tests, 0 errors/failures, and 7 skipped;
- live contract sampled ten `rois0` messages at about 15.02 Hz;
- five real detections occurred in the sampled outdoor scene;
- invalid/out-of-image ROI count was zero after clamping both corners.

The detections prove inference ran; they do not by themselves prove classification accuracy.

### Lucid driver

- final package rebuild succeeded with zero build stderr;
- installed modified library SHA-256 at the time of validation:
  `b38659ec42ae378718d6354d78d6aafd951df58fab1092d419ea748ebc5e7df3`;
- real-camera composable-container SIGINT stress passed 3/3;
- one run remained active for 319 seconds at approximately 14.8–15 Hz before clean shutdown;
- shutdown logs contained the pre-shutdown gate, `StopStream`, deregister, delete, destroy, and
  close markers with no SIGSEGV or heap error.

Important qualification: the Lucid package's last recorded `colcon test` result is not clean and
predates the final 21:23 source edits. Canonical aggregation reports 159 tests, 0 errors, 147
failures, 7 skipped; six of eight CTest lint/schema targets failed. The failures are concentrated in
copyright, cpplint, flake8, lint_cmake, uncrustify, and xmllint; pep257 passed and cppcheck was
skipped. Therefore this release claims successful compilation and real-device runtime stress, not
a clean Lucid unit/lint suite.

After the final commit review, two small correctness cleanups were applied in the v1.1 candidate:
CameraInfo now copies the original callback header instead of reading an image message after move,
and the launch forwards each flip parameter from YAML exactly once. The candidate was rebuilt in an
isolated build tree and tested against serial 214000332 on isolated Domain 219 for three cycles.
Every cycle received a 1920x1200 CameraInfo with frame
`traffic_light_camera/camera_link`, exited with code 0, emitted all six ordered SDK shutdown
markers, and produced zero crash/context/process-died/error matches. Those targeted runtime cycles
validate the post-review delta; they do not replace the still-failing package lint suite.

### topic_tools overlay

- base: official tag 1.1.2, commit `0fc927b7c0af0aaffae34a947b6ea4a7f9f97c94`;
- patched relay library SHA-256:
  `45ad17f6e202a74cba025ad94f219f7402ef532551cd1b8162923267a16cb9e3`;
- overlay build/test return codes were zero; aggregated artifacts report 121 tests, 0
  errors/failures, and 23 skipped;
- helper verified the selected package prefix and `ldd` resolution before launch;
- final cycle had zero relay `RCLError`, context-invalid abort, or process-died event.

## Final guarded standalone cycle

Run ID: `20260811_184357`, started at approximately 18:43 KST.

Original local ROS log SHA-256:
`e250a90fd80d514912eccf5706bc7a485dad6f0344ea873c0e919db1d7e0aeb4`.

| Check | Observation | Result |
|---|---|---|
| camera CreateDevice / StartStream | serial 214000332, one each | PASS |
| image / CameraInfo dimensions | 1920x1200 / 1920x1200 | PASS |
| image / info frame | `traffic_light_camera/camera_link` | PASS |
| image / info stamp contract | matched | PASS |
| image rate | about 15.15 Hz | PASS |
| CameraInfo rate | about 15.01 Hz | PASS |
| calibration fx | 1049.272996 | PASS against configured contract |
| generic YOLO output | ten samples, about 15.02 Hz | PASS |
| real YOLO detections | five in sampled messages | PASS for inference activity |
| out-of-frame ROI | zero | PASS |
| YOLO input wiring | decompressor to `/tensorrt_yolox` | PASS |
| YOLO output owner | one `/tensorrt_yolox` publisher | PASS |
| fusion consumers | ROI pointcloud, cluster, detected-object fusion = 3 | PASS |
| TLR component count | fine detector, two classifiers, visualizer = 4/4 | PASS presence only |
| RViz | child of the same top launch | PASS |
| new local control/vehicle/CAN publisher | zero | PASS for PC2 scope |
| clean stop | all children stopped in 1.559 s | PASS |
| post-stop PC2 nodes/publishers | zero above baseline | PASS |
| PID/lock/exact route residue | zero | PASS |
| process-died / TensorRT / relay / camera memory errors | zero | PASS |

This final cycle used `REQUIRE_PC3_INPUTS=0`. At its snapshot, publisher counts for PC3 LiDAR,
vector map, pointcloud map, `/tf`, and `/tf_static` were zero. It is consequently a PC2 standalone
pipeline PASS, not an end-to-end distributed semantic PASS.

## Earlier three-PC observations

A separate persistent outdoor probe, while PC1/PC3 were visible, observed:

| Flow | Observation |
|---|---|
| PC3 pointcloud-before-sync | 228 messages / 30 s, header-derived 10.000 Hz |
| PC3 raw-ex pointcloud | 228 messages / 30 s, header-derived 10.000 Hz |
| PC2 compressed camera | 450 messages / 30 s, 14.971 Hz |
| PC2 normalized camera info | 450 messages / 30 s, 14.971 Hz |
| PC2 official objects | 273 messages / 30 s, 9.091 Hz, all arrays empty |
| static TF | two cached transient-local messages, ten static pairs |
| dynamic TF | zero messages in the bounded window |

PC1 planning/AEB/API subscriptions matched PC2's official objects, and planning subscriptions
matched the PC2 traffic-signal endpoint with compatible QoS. Endpoint/GID attribution identified
PC1 at `.2`, PC2 at `.110`, and PC3 at `.7`. These observations prove DDS transport and endpoint
matching for that window, not callback-level PC1 processing or semantic correctness.

The same campaign found the PC3 map projector origin did not match C_track and the NovAtel INS
orientation was invalid. Dynamic `map -> base_link` and `/localization/kinematic_state` remained
absent. These are upstream blockers, not PC2 camera/YOLO initialization failures.

## Known remaining defect

PC2's obstacle-segmentation pointcloud was BEST_EFFORT while the PC1
`obstacle_cruise_planner` requested RELIABLE in an earlier three-PC snapshot. That exact
subscription is incompatible and must be aligned and retested.

## Required final acceptance

Before driving or recording paper data:

1. PC3: verify GNSS, IMU, live LiDAR rates, correct C_track projector, valid orientation,
   localization state, clock synchronization, and continuous `map -> base_link`.
2. PC1/PC3 remain online: run five PC2 cycles with `REQUIRE_PC3_INPUTS=1`.
3. During the run, measure payloads rather than endpoint counts: nonempty scene-correct fused
   objects, rough/fine TLR ROIs, final traffic signals, and PC1 receipt.
4. Resolve the obstacle pointcloud QoS mismatch.
5. Audit duplicates and physical actuation again after enabling PC1 CAN write.
6. Add PC4, verify bridge direction/topic ownership and timestamps, then perform a stationary soak
   before any low-speed closed-course test.

The validator has bounded DDS calls but its repeated sequential discovery operations can take
longer than the apparent loop count during a network failure. Interrupt it with SIGINT only after
confirming the guarded launcher cleans its process group; always inspect PID, lock, route and graph
state afterward.
