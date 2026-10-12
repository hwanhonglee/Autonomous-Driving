# Resource guard addendum: still not executed

<!-- HH_261001 - Supersede the unexecuted resource plan without deleting failed or historical evidence. -->

The first prospective declaration remains unchanged and unexecuted at `storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_plan_v1.json`, SHA256 `44300295767c8dbfdcf5cf8e72a0e669c8ce3793908c6d9d6f6d296047502a76`. Its three exact source files are archived byte-identically in `town07_low_speed_moving_declared_v1_source/`; their hashes match the first plan/note. It is superseded for execution, not retroactively edited.

The active new declaration is `storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_plan_v2_resource_guard.json`, SHA256 `b8116231db1a5a548b1e890f21d403feb50bcef9403b49cbdcba5e3d055dfee1`. The fixed40–120 m task, controls, seed, all-anchor membership, history/future, QA, and PENDING_QA scope are unchanged.

Resource-only additions:

- Owner preflight requires **5.00 GiB floor +0.50 GiB estimated maximum raw capture**, exactly5,905,580,032 free bytes.
- Every observed native tick records current free disk bytes and fails the task before another normal command if space falls below **5 GiB +32 MiB closing-journal reserve**, using the existing abort/full-brake/cleanup path. No emergency brake is suppressed, no record deleted, and no data automatically transferred.
- Independent replay uses the recorded per-tick disk observation, not current free space. The owner separately records final free bytes and fails if the5 GiB floor is not preserved. Other processes can still consume space; the guard is an observed abort policy, not an absolute filesystem reservation.

At final preflight, available space was5,867,048,960 bytes (about5.464 GiB), **38,531,072 bytes below** the5.5 GiB requirement. The owner correctly refuses before simulator launch. Await the user's offload direction; no old artifacts were deleted. The packaged API/asset dry-run passed before this resource addition, and the guarded worker's full CLI/source-plan validation passes. Nine task/control/resource tests pass; fifteen with body/census tests.

The launch command remains the one in the original plan note, but now uses the new declared resource-guard plan and cannot pass until disk budget is available. No capture output directory exists. No qualification, Common10 conversion, admission, or training was performed.
