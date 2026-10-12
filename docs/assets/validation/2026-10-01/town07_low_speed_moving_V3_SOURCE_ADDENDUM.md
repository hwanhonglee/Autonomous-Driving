# HH_261001 — Prospective source-only V3 moving capture supplement

No capture has run. This supplement closes the known external inherited
BasicAgent source-evidence gap **before the first capture**. It does not alter
the expert controller, route, 40–120 m fixed window, timing, history/future,
native/future/body gates, 5 GiB floor, 0.50 GiB budget, or PENDING admission scope.
It is not a third driving trial or a new control arm.

The unchanged V2 task/resource declaration remains byte-identical, SHA256
`b8116231db1a5a548b1e890f21d403feb50bcef9403b49cbdcba5e3d055dfee1`.
Its three original source files and older unexecuted declarations remain intact.
V2-only entry is superseded prospectively by this V3 source wrapper; the underlying
fixed task contract is supplemented, not silently rewritten.

New declaration: `storage/experiments/validation/2026-10-01/town07_low_speed_moving_v3_external_sources_addendum.json`,
SHA256 `d25e834f697670320317cec286dbe57f942df869f08b85b90ae425cd5c0f0574`.
The owner and worker both require this externally supplied SHA. The new declaration
pins the three wrapper/helper files and eight allowlisted package Python files.

## Evidence sequence

1. Original V2 preflight runs unchanged, including disk budget and CLI checks.
2. Before the original owner starts any server process, archive the eight fixed
   source files (69,282 bytes total), verify each copied SHA and source SHA, and
   write the original owner plan plus V3 addendum binding.
3. The worker imports the original package BasicAgent, LocalPlanner,
   GlobalRoutePlanner, VehiclePIDController and `agents.tools.misc`; verify actual
   imported paths, class identity and transitive class aliases. Record all file
   hashes before entering the unchanged worker's temporary subclass scope.
4. Run the original expert worker with exactly its previous arguments. Only a
   new wrapper module and its consumed source-addendum SHA argument are added.
5. After original worker cleanup, repeat source/import/archive checks. The
   original worker return/error remains intact. Postcheck failure cannot become
   capture success. The owner verifies worker pre/post records, source archive and
   current source hashes again; its original result is never overwritten.

Only regular files under the fixed
`/home/a/carla-autoware-universe/CARLA_0.9.15/PythonAPI/carla/agents` root are allowed:
three `__init__.py` files, `navigation/basic_agent.py`, `local_planner.py`,
`global_route_planner.py`, `controller.py`, and `tools/misc.py`. Symlinks, redirected
paths, unlisted sources, overwritten classes and archive membership changes fail.
There is no recursive arbitrary directory copy. No package source is edited,
installed or deleted. This binds that package source subset, not all Python/OS/
compiled dependencies; the original pinned CARLA egg/assets remain unchanged.

The frozen V1 numerical auditor deliberately remains unchanged and will still
state its original external-source limitation. A V3
`external_agents_owner_after.json` PASS is a separate hash-bound supplement
resolving that specific limitation for this V3 capture; it does not rewrite the
older report, grant admission, or replace any numerical/raw-data check.

## Frozen new source hashes

| File | SHA256 |
| --- | --- |
| `tools/e2e/town07_moving_external_sources_v3_20261001.py` | `d33cd5ac2831124fbd1dd5283c036e969830d3b6921c0654795ef73b5e208c1d` |
| `tools/e2e/collect_town07_low_speed_moving_v3_20261001.py` | `b26a4ee76d8a4606e52e545a5296af629d536e73640750a87b1e6574251a9f53` |
| `tools/e2e/run_owned_town07_low_speed_moving_v3_20261001.py` | `2079c4daf797a7bad709c2677b13c713f8f5a9851a8af9fedbe3581231695d39` |
| `learning/tests/test_town07_moving_external_sources_v3_20261001.py` | `90757bda4a69e5c60327428c6a14445172683a5b7291547560e49155c52db27f` |

41 combined tests pass, including actual package imports without a CARLA client,
source/class shadowing, archive corruption, unsafe paths, causal controls, fixed
window checks and original source hashes. The owner test proves the archive exists
before a simulated server-start boundary. This test never starts a server.

## Parent-reviewed execution only

Dry-run uses the following command without `--execute`. Future capture, only after
parent review and sufficient disk space:

```sh
OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 python3 tools/workspace_python.py \
  -m scripts.e2e.run_owned_town07_low_speed_moving_v3_20261001 -- \
  --addendum-sha256 d25e834f697670320317cec286dbe57f942df869f08b85b90ae425cd5c0f0574 \
  --execute
```

Actual dry-run refused capture on 2026-10-01 with free bytes 5,838,696,448 versus
required 5,905,580,032 (short 66,883,584). No simulator, capture output, GPU job,
offload or deletion was started. Space must be resolved separately without
lowering the original thresholds. The capture path remains the unchanged
`storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v1`.
