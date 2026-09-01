from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import textwrap
import time

import pytest


ROOT = Path(__file__).parents[1]
RUNNER = ROOT / "scripts/e2e/run_autoware_vad_town_matrix.sh"
WORKSPACE_LOCK = ROOT / "scripts/e2e/workspace_runtime_lock.sh"


FAKE_MATRIX_TOOL = r"""
from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import sys


args = sys.argv[1:]
command = args[0]


def option(name: str, default: str | None = None) -> str | None:
    try:
        return args[args.index(name) + 1]
    except (IndexError, ValueError):
        return default


output_root_value = option("--output-root")
if output_root_value is None:
    raise SystemExit("missing --output-root")
output_root = Path(output_root_value)
status_path = output_root / "maps/town01/status.json"


def read_status() -> dict:
    return json.loads(status_path.read_text(encoding="utf-8"))


def write_status(value: dict) -> None:
    status_path.write_text(json.dumps(value), encoding="utf-8")


def summarize() -> None:
    status = read_status()
    aggregate_row = copy.deepcopy(status)
    override = os.environ.get("FAKE_AGGREGATE_MAP_STATUS")
    if override:
        aggregate_row["status"] = override
    aggregate_status = {
        "PASS": "COMPLETE",
        "FAILED": "FAILED",
    }.get(aggregate_row["status"], "INCOMPLETE")
    aggregate = {
        "status": aggregate_status,
        "maps": [aggregate_row],
    }
    (output_root / "aggregate.json").write_text(
        json.dumps(aggregate), encoding="utf-8"
    )


if command == "prepare":
    raise SystemExit(0)
if command == "list-runnable":
    print("town01\tTown01\tfixture\t/full/map\tpackaged_town")
    raise SystemExit(0)
if command == "validate-trial":
    trial_dir = Path(option("--trial-dir", ""))
    raise SystemExit(0 if (trial_dir / "matrix_validation.json").is_file() else 1)
if command == "summarize":
    summarize()
    raise SystemExit(0)
if command != "update":
    raise SystemExit(f"unsupported fake command: {command}")

status = read_status()
trial_id = option("--trial-id")
reason = option("--reason")
stage = option("--stage")
if trial_id is not None:
    trial = status["trials"][trial_id]
    trial["status"] = option("--trial-status")
    trial["reason"] = reason
    trial["attempt_directory"] = option(
        "--attempt-dir", trial.get("attempt_directory")
    )
    trial["validation"] = option("--validation", trial.get("validation"))
    states = [status["trials"][name]["status"] for name in ("straight", "turn")]
    if all(value == "PASS" for value in states):
        status["status"] = "PASS"
        status["stage"] = "complete"
        status["reason"] = "Straight and turn full-stack trials both passed."
    elif "FAILED" in states:
        status["status"] = "FAILED"
        status["stage"] = stage
        status["reason"] = reason
    else:
        status["status"] = "RUNNING"
        status["stage"] = stage
        status["reason"] = reason
else:
    new_status = option("--status")
    if new_status == "FAILED":
        for name in ("straight", "turn"):
            trial = status["trials"][name]
            if trial["status"] in {"PENDING", "RUNNING"}:
                trial["status"] = "FAILED"
                trial["reason"] = (
                    "Not executed because map-level prerequisite failed: " + reason
                )
                trial["validation"] = None
    status["status"] = new_status
    status["stage"] = stage
    status["reason"] = reason

write_status(status)
summarize()
if (
    os.environ.get("FAKE_FAIL_RUNNING_UPDATE") == "1"
    and trial_id is None
    and option("--status") == "RUNNING"
):
    raise SystemExit(17)
"""


def _write_executable(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(textwrap.dedent(value).lstrip(), encoding="utf-8")
    path.chmod(0o755)


def _status_payload(
    output_root: Path,
    map_status: str,
    trial_status: str,
) -> dict:
    trials = {}
    for trial_id in ("straight", "turn"):
        attempt = (
            output_root
            / "maps/town01/trials"
            / trial_id
            / "attempt_001"
        )
        attempt.mkdir(parents=True)
        (attempt / "matrix_validation.json").write_text("{}", encoding="utf-8")
        trials[trial_id] = {
            "status": trial_status,
            "reason": "fixture pre-state",
            "attempt_directory": str(attempt),
            "validation": (
                str(attempt / "matrix_validation.json")
                if trial_status == "PASS"
                else None
            ),
        }
    return {
        "map_id": "town01",
        "canonical_name": "Town01",
        "runnable": True,
        "status": map_status,
        "stage": "fixture_pre_state",
        "reason": "fixture pre-state",
        "trials": trials,
    }


def _fixture_repo(
    tmp_path: Path,
    *,
    map_status: str,
    trial_status: str,
) -> tuple[Path, Path, Path]:
    repo = tmp_path / "repo"
    scripts = repo / "scripts/e2e"
    scripts.mkdir(parents=True)
    runner = scripts / RUNNER.name
    shutil.copy2(RUNNER, runner)
    shutil.copy2(WORKSPACE_LOCK, scripts / WORKSPACE_LOCK.name)
    _write_executable(scripts / "autoware_vad_town_matrix.py", FAKE_MATRIX_TOOL)
    _write_executable(scripts / "env.sh", "#!/usr/bin/env bash\n")
    _write_executable(
        scripts / "apply_mission_planner_lane_only_patch.sh",
        "#!/usr/bin/env bash\nexit 0\n",
    )
    _write_executable(
        scripts / "mission_planner_build_provenance.py",
        "#!/usr/bin/env python3\nraise SystemExit(0)\n",
    )
    _write_executable(
        scripts / "apply_vad_object_safety_patches.sh",
        "#!/usr/bin/env bash\nexit 0\n",
    )
    _write_executable(
        scripts / "vad_object_safety_build_provenance.py",
        "#!/usr/bin/env python3\nraise SystemExit(0)\n",
    )
    _write_executable(
        scripts / "process_group_cleanup.sh",
        """
        #!/usr/bin/env bash
        e2e_stop_owned_process_group() {
          kill -TERM -- "-$1" 2>/dev/null || true
          wait "$2" 2>/dev/null || true
          return 0
        }
        """,
    )
    _write_executable(
        scripts / "run_carla_map.sh",
        """
        #!/usr/bin/env bash
        sleep 30
        """,
    )
    _write_executable(
        scripts / "prepare_carla_expert_route_catalog.py",
        "#!/usr/bin/env python3\nraise SystemExit(99)\n",
    )
    _write_executable(
        scripts / "run_recorded_route_trial.sh",
        "#!/usr/bin/env bash\nexit 99\n",
    )

    output_root = tmp_path / "matrix"
    (output_root / "maps/town01").mkdir(parents=True)
    route_generation_contracts = {
        trial_id: {
                "weather": "ClearNoon",
                "seeds": [0],
                "pairs_per_seed": 1,
                "minimum_distance_m": 20.0,
                "maximum_distance_m": 100.0,
                "preferred_distance_m": 60.0,
                "sampling_resolution_m": 1.0,
                "maximum_endpoint_offset_m": 2.0,
                "maximum_traces_per_scenario": 10,
            }
        for trial_id in ("straight", "turn")
    }
    plan = {
        "runtime_profile": {
            "ready_timeout_sec": 1,
            "wrapper_options": ["--fixture"],
        },
        "route_generation_contracts": route_generation_contracts,
        "maps": [
            {
                "map_id": "town01",
                "route_generation_contracts": route_generation_contracts,
            }
        ],
    }
    (output_root / "matrix_plan.json").write_text(
        json.dumps(plan), encoding="utf-8"
    )
    status = _status_payload(output_root, map_status, trial_status)
    (output_root / "maps/town01/status.json").write_text(
        json.dumps(status), encoding="utf-8"
    )
    (output_root / "maps/town01/route_matrix.json").write_text(
        "{}", encoding="utf-8"
    )
    return repo, runner, output_root


def _run_resume(
    repo: Path,
    runner: Path,
    output_root: Path,
    env_override: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env.update(env_override or {})
    return subprocess.run(
        [
            str(runner),
            str(output_root),
            "--runtime-profile",
            "speed_30kph",
            "--maps",
            "town01",
            "--resume",
        ],
        cwd=repo,
        env=env,
        text=True,
        capture_output=True,
        timeout=15,
        check=False,
    )


@pytest.mark.parametrize(
    ("map_status", "trial_status"),
    [("FAILED", "FAILED"), ("RUNNING", "RUNNING")],
)
def test_resume_fast_path_recovers_strict_trials_to_map_pass(
    tmp_path: Path,
    map_status: str,
    trial_status: str,
) -> None:
    repo, runner, output_root = _fixture_repo(
        tmp_path,
        map_status=map_status,
        trial_status=trial_status,
    )

    result = _run_resume(repo, runner, output_root)

    assert result.returncode == 0, result.stderr
    assert "status recovered" in result.stdout
    assert "FINAL_STATE_PASS selected_maps=1" in result.stdout
    status = json.loads(
        (output_root / "maps/town01/status.json").read_text(encoding="utf-8")
    )
    assert status["status"] == "PASS"
    assert status["stage"] == "complete"
    assert {trial["status"] for trial in status["trials"].values()} == {"PASS"}


def test_unexpected_set_e_exit_terminalizes_active_map(tmp_path: Path) -> None:
    repo, runner, output_root = _fixture_repo(
        tmp_path,
        map_status="PENDING",
        trial_status="PENDING",
    )

    result = subprocess.run(
        [str(runner), str(output_root), "--maps", "town01"],
        cwd=repo,
        env={**os.environ, "FAKE_FAIL_RUNNING_UPDATE": "1"},
        text=True,
        capture_output=True,
        timeout=15,
        check=False,
    )

    assert result.returncode == 17
    status = json.loads(
        (output_root / "maps/town01/status.json").read_text(encoding="utf-8")
    )
    assert status["status"] == "FAILED"
    assert status["stage"] == "unexpected_exit"
    assert "status 17" in status["reason"]
    assert {trial["status"] for trial in status["trials"].values()} == {"FAILED"}
    aggregate = json.loads(
        (output_root / "aggregate.json").read_text(encoding="utf-8")
    )
    assert aggregate["status"] == "FAILED"


def test_final_selected_map_check_rejects_stale_failed_aggregate(
    tmp_path: Path,
) -> None:
    repo, runner, output_root = _fixture_repo(
        tmp_path,
        map_status="FAILED",
        trial_status="FAILED",
    )

    result = _run_resume(
        repo,
        runner,
        output_root,
        {"FAKE_AGGREGATE_MAP_STATUS": "FAILED"},
    )

    assert result.returncode == 1
    assert "selected-map final-state validation failed" in result.stderr
    status = json.loads(
        (output_root / "maps/town01/status.json").read_text(encoding="utf-8")
    )
    assert status["status"] == "PASS"


def test_signal_keeps_interrupted_terminalization_contract(tmp_path: Path) -> None:
    repo, runner, output_root = _fixture_repo(
        tmp_path,
        map_status="PENDING",
        trial_status="PENDING",
    )
    process = subprocess.Popen(
        [
            str(runner),
            str(output_root),
            "--maps",
            "town01",
            "--startup-timeout-sec",
            "30",
        ],
        cwd=repo,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    status_path = output_root / "maps/town01/status.json"
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        status = json.loads(status_path.read_text(encoding="utf-8"))
        if status["status"] == "RUNNING":
            break
        time.sleep(0.05)
    else:
        process.kill()
        process.wait(timeout=5)
        raise AssertionError("runner did not enter the active-map state")

    os.kill(process.pid, signal.SIGTERM)
    stdout, stderr = process.communicate(timeout=15)

    assert process.returncode == 130, (stdout, stderr)
    status = json.loads(status_path.read_text(encoding="utf-8"))
    assert status["status"] == "FAILED"
    assert status["stage"] == "interrupted"
    assert status["reason"] == (
        "Matrix interrupted; only owned process groups were stopped."
    )
    assert {trial["status"] for trial in status["trials"].values()} == {"FAILED"}
