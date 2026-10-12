"""HH_261001 - Own one fresh full-route expert start/goal-stop measurement; never launch on import or preflight."""

import argparse
import json
from pathlib import Path
import sys

from scripts.e2e import run_owned_ah_town07_research_20261001 as lifecycle
from scripts.e2e import collect_town07_low_speed_start_20261001 as worker


OUTPUT = lifecycle.ROOT / "storage/experiments/validation/2026-10-01/town07_low_speed_start_expert_v1"


def command():
    return [sys.executable, str(lifecycle.ROOT / "tools/workspace_python.py"), "-m",
        "scripts.e2e.collect_town07_low_speed_start_20261001", "--", str(OUTPUT / "episode"),
        str(lifecycle.ROUTE), "--host", "127.0.0.1", "--port", str(lifecycle.PORT),
        "--control-transport", "acknowledged_batch", "--agent-initialization", "after_bootstrap",
        "--stationary-warmup-sec", "3.5", "--stationary-tail-sec", "6.5", "--max-duration-sec", "180",
        "--target-speed-kmh", "9", "--seed", "2026100101", "--weather", "ClearNoon",
        "--mapping", str(lifecycle.MAPPING), "--calibration", str(lifecycle.CALIBRATION),
        "--spawn-z-offset-m", "0.5", "--expected-live-xodr-sha256", lifecycle.PINS[lifecycle.XODR],
        "--goal-stop-profile", "comfortable_v2", "--goal-tolerance-m", "1",
        "--basic-agent-base-min-distance-m", "2", "--basic-agent-distance-ratio", "0.2"]


def prepare():
    lifecycle.OUTPUT = OUTPUT
    lifecycle.PINS = {path: sha for path, sha in lifecycle.PINS.items()
                      if path not in (lifecycle.WEIGHTS, lifecycle.METADATA)}
    lifecycle.collector_command = command
    plan = lifecycle.preflight()
    argv = command()
    worker.parse_args(argv[argv.index(str(OUTPUT / "episode")):])
    plan.update(schema="carla.owned_town07_low_speed_start.v1", source_checkpoint_sha256=None,
        learned_control=False, capture_budget_seconds=180, wall_timeout_seconds=240,
        restart_demonstration=False, training_data_approved=False,
        interpretation="One fixed full-route expert physical launch and measured goal-stop attempt; failures retained; no automatic dataset admission.",
        normal_control_policy=dict(launch_throttle=.13, measured_handoff_mps=.5, maximum_launch_seconds=8.,
            post_handoff_ramp_per_second=.05, post_handoff_cap=.20, normal_brake_cap=.15,
            original_basic_agent_emergency_unchanged=True),
        scalar_quality="original comfortable_v2 native20Hz and actual10Hz camera checks unchanged",
        future_geometry_quality="original full-window gates still required; no admission from scalar QA alone")
    return plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    plan = prepare()
    if not args.execute:
        print(json.dumps(plan, indent=2))
        return 0
    lifecycle.execute(plan)
    owner = json.loads((OUTPUT / "owner_result.json").read_text())
    recorded = json.loads((OUTPUT / "owner_plan.json").read_text())
    source_match = all(lifecycle.digest(lifecycle.ROOT / path) == digest
        and lifecycle.digest(OUTPUT / "source" / path) == digest for path, digest in recorded["source_sha256"].items())
    port_errors = []
    try:
        lifecycle.ports_free(lifecycle.PORT)
        ports_released = True
    except OSError as error:
        ports_released = False
        port_errors.append(str(error))
    episode = OUTPUT / "episode"
    if not episode.exists():
        episode = OUTPUT / "episode.partial"
    manifest_path = episode / "manifest.json"
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    quality = manifest.get("result", {}).get("goal_stop_quality")
    result = dict(schema="carla.owned_town07_low_speed_start_outcome.v1", owner=owner,
        source_hashes_match=source_match, port_triplet_released=ports_released, port_check_errors=port_errors,
        capture_status=manifest.get("status"), capture_error=manifest.get("error"),
        goal_reached=manifest.get("result", {}).get("goal_reached", False), original_scalar_quality=quality,
        full_future_geometry_approved=False, training_data_approved=False, learned_control=False,
        original_failed_evidence_preserved=True)
    lifecycle.write_json(OUTPUT / "owner_expert_result.json", result)
    print(json.dumps(result, indent=2))
    return 0 if (source_match and ports_released and not owner["cleanup_errors"]
                 and owner["collector_exit_code"] == 0 and quality and quality["status"] == "PASS") else 1


if __name__ == "__main__":
    raise SystemExit(main())
