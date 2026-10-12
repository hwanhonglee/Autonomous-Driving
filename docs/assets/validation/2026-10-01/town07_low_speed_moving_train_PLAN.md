# One prospective Town07 low-speed moving TRAIN candidate

<!-- HH_261001 - New explicit scope and fresh pending metadata; no legacy denial/goal override. -->

Prepared and frozen; simulator NOT launched by this work. This is one bounded expert collection, not training admission. Its intended labels are sustained low-speed moving context, never goal completion, launch, or stopping. The existing sealed 15 kph moving qualification does not accept this new slot. Any later conversion requires a separately reviewed, versioned qualification and explicit authorization; none is implemented here.

## Fixed prospective task

- Slot `T07-LOW-CRUISE-20261001-V1`; same already-TRAIN Town07 straight route, unchanged route SHA `8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9`. This is not an independent route or held-out test.
- Inclusive independent rear-reference geometric anchor interval **40–120 m**, every camera anchor retained. First native progress >120 m permanently closes membership. Require ≥30 s actual selected-camera span. No model-score selection, post-capture trimming, or target clipping.
- Ten complete preceding/current camera observations (0.9 s) with a preceding native guard, 64 measured future points (6.4 s), native 20 Hz and six-camera 10 Hz, 70 warmup states, and 130 tail states. Selected futures end before tail.
- BasicAgent expert; 9 km/h target, normal brake cap 0, throttle cap 0.20, initial measured-speed launch throttle 0.13 until 0.5 m/s, eight-second launch timeout, post-handoff throttle-cap ramp 0.05/s, zero normal steering through 0.75 m/s. Normal target ramps at 1.5 m/s² to 2.5 m/s after the original initial-drive proposal. BasicAgent emergency returns remain unmodified and outside those normal limits.
- Packaged CARLA Town07, Epic rendering, Prius, ClearNoon, seed 2026100101, port 2440–2442. One attempt, no automatic retry, ≤180 s simulation and 240 s collector wall budget. Full JPEG/native/ACK/policy journals retained; no exact NN tensors because no NN runs.

The new worker uses the existing collector's `goal_stop_profile=disabled` moving-task pathway. It inserts the same v2 normal-control implementation in a dedicated BasicAgent/LocalPlanner wrapper. This is NOT the identical whole v2 collector: catalog-stop governor/terminal-plan completion is not used, and original moving-task full braking begins only after all selected future support, followed by 130 recorded tail states. That tail is outside the label scope and is not comfortable-stop supervision. A target-ramp test proves equality to the original governor while far from the goal, after the initial proposal; a separate wrapper test verifies full emergency braking survives normal caps.

Why this integration is needed: enabling the old comfortable goal profile would unconditionally fail goal-stop QA even after legitimate moving-task completion with `goal_reached=false`. The worker does not fake that QA, suppress its exception, or modify its validator. Instead it reports original whole-recording goal/native QA separately under `whole_recording_goal_stop_quality_diagnostic`, preserving any FAIL. Fresh native metadata is prospectively PENDING_QA; no old recording, denial marker, or failed manifest is read or normalized.

## Qualification remains separate and fail-closed

The entire fixed selected set and all history/native guard/future intervals must pass the original nine 10 Hz physical bounds, native scalar speed/±2.9 m/s² checks, camera pixels/calibration/frame timing, ACK snapshot/control chain, actor body and bounded sweeps within the unchanged 10 cm lane inset, and absence of events. A failure anywhere rejects qualification of the complete task window, rather than shrinking it. All outside-window data and whole-recording quality failures remain archived and reported. Task completion is not proof of any of these quality gates and is not admission.

The historical v2 capture suggests entry at40 m after24.20 s, exit120 m after56.85 s, then6.4 s future+6.5 s tail: approximately69.75 s simulated recording, not a guarantee. The old JPEG payload rate predicts roughly0.415 GiB for700 camera anchors; budget approximately0.42–0.50 GiB including journals, not a hard≤0.4 GiB claim. The owner retains its existing ≥5 GiB startup-free-space requirement. Main risks are launch pulse outside the label scope, tail brake discontinuity outside label scope, new-wrapper/source integration, or an unexpected in-window physical/sensor/body failure. None becomes a success claim or triggers automatic retry.

## Frozen artifacts and command

Native prospective plan: `storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_plan_v1.json`, SHA256 `44300295767c8dbfdcf5cf8e72a0e669c8ce3793908c6d9d6f6d296047502a76`.

| New file | SHA256 |
|---|---|
| `tools/e2e/town07_low_speed_moving_task_20261001.py` | `566c5e2e0cab560b00b0add27fe139ac4860c6cc2756cca728974ee0f97e591e` |
| `tools/e2e/collect_town07_low_speed_moving_20261001.py` | `a4645c1c5c59fa2e760b337ad6bbb85a755b2ef007395341b891f4c5f38f7872` |
| `tools/e2e/run_owned_town07_low_speed_moving_20261001.py` | `467d8aab7165cc7f9ce585738eb1a39b874648780660d8a4dbd3280af77d879e` |
| `learning/tests/test_town07_low_speed_moving_20261001.py` | `7f51bc6f565a1a1629c92ee7951044775ada473e9ef3991c1d2dc394fba0e850` |

Seven focused task/control tests pass, thirteen including the body/census tests. Real packaged API/asset hashes and the complete worker arguments pass dry-run preflight, with `simulator_started=false`. Original shared collector, old moving authority, v2 capture files, and validators remain unchanged.

Review/preflight only:

```sh
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 tools/workspace_python.py -m scripts.e2e.run_owned_town07_low_speed_moving_20261001
```

After parent review only, the complete explicit launch command is:

```sh
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 tools/workspace_python.py -m scripts.e2e.run_owned_town07_low_speed_moving_20261001 -- --execute
```

Fresh output: `storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v1`. Do not rerun `--declare`: the immutable prospective plan already exists. No capture was started here.
