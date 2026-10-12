#!/usr/bin/env python3
"""HH_260906 - Freeze a new two-case terminal approach experiment without reopening old attempts or changing stop/data gates."""
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path.insert(0, str(ROOT))
from scripts.e2e.carla_goal_stop_profile import TerminalApproachDevelopmentGoalStopConfig

OLD = ROOT / "artifacts/training/2026-09-15/native_boundary_and_substeps/substep_abba"


def sha(path):
    assert path.is_file() and not any(p.is_symlink() for p in (path, *path.parents)), path
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert sha(OLD / "plan.json") == "dcfb17fa955208879606dce4c618ef0720522f780e046d4c5fde3aeabfd937e0"
    old = json.loads((OLD / "plan.json").read_text())
    changed = {"scripts/e2e/" + name for name in ("carla_goal_stop_profile.py", "carla_physics_substeps.py", "collect_carla_vad_expert.py")}
    for category in ("source_sha256", "input_sha256", "audit_math_sha256", "orchestration_sha256", "analysis_sha256"):
        for name, pin in old[category].items():
            path = BASE / "prior_sources" / Path(name).name if name in changed else ROOT / name
            assert sha(path) == pin, name
    for name, pin in json.loads((BASE / "prior_publication_pins.json").read_text()).items():
        assert sha(ROOT / name) == pin, name
    cases = []
    for name, arm in (("01_coast_A", "coast"), ("02_feedback_B", "feedback")):
        command = old["cases"][0]["command"].copy()
        command[2] = str((BASE / name).relative_to(ROOT))
        command[command.index("--goal-stop-profile") + 1] = "comfortable_v9_terminal_feedback"
        assert command[command.index("--physics-substep-profile") + 1] == "reference_10ms"
        assert command[command.index("--hold-brake-mode") + 1] == "immediate"
        command.extend(["--terminal-approach-mode", arm])
        assert not (BASE / name).exists()
        cases.append(dict(id=name, arm=arm, command=command, maximum_attempts=1))
    source_pins = {name: sha(ROOT / name) for name in old["source_sha256"]}
    orchestration = {"scripts/e2e/probe_carla_port_readiness.py": sha(ROOT / "scripts/e2e/probe_carla_port_readiness.py")}
    for name in ("run_case.py", "render_evidence.py"):
        orchestration[str((BASE / name).relative_to(ROOT))] = sha(BASE / name)
    analysis = {name: sha(ROOT / name) for name in (
        "scripts/e2e/summarize_carla_terminal_approach.py", "scripts/e2e/summarize_carla_hold_substep_abba.py",
        "scripts/e2e/summarize_carla_hold_brake_ramp_abba.py", "scripts/e2e/summarize_carla_low_speed_steering_abba.py")}
    keys = ("control_transport", "agent_initialization", "target_speed_kmh", "physics_hz", "camera_hz",
            "stationary_warmup_seconds", "required_measured_stop_dwell_seconds", "stationary_tail_seconds",
            "maximum_total_simulation_seconds", "collector_wall_timeout_seconds", "quality", "map", "scenario",
            "vehicle_type", "wheelbase_m", "spawn_z_offset_m", "seed", "weather", "screenshot_fallback")
    plan = {key: old[key] for key in keys}
    plan.update(schema="carla.terminal_approach_plan.v1", comment=__doc__,
        declared_at_utc=datetime.now(timezone.utc).isoformat(), campaign_id="town07_terminal_approach_v9",
        cases=cases, maximum_attempts_per_revision=2, automatic_retry=False,
        source_sha256=source_pins, input_sha256=old["input_sha256"], audit_math_sha256=old["audit_math_sha256"],
        analysis_sha256=analysis, orchestration_sha256=orchestration,
        protocol_configuration_by_arm={arm: asdict(replace(TerminalApproachDevelopmentGoalStopConfig(), terminal_approach_mode=arm)) for arm in ("coast", "feedback")},
        physics_profile_by_arm={arm: "reference_10ms" for arm in ("coast", "feedback")},
        required_original_substeps=dict(substepping=True, max_substep_delta_time=.01, max_substeps=10),
        requested_max_substep_delta_time_by_arm={arm: .01 for arm in ("coast", "feedback")},
        fixed_max_substeps=10, explicit_settings_readback_required=True,
        changed_factor="Only the explicit coast/feedback normal terminal-throttle policy; fixed reference_10ms, 20 Hz state/control, 10 Hz cameras, immediate hold and zero low-speed steering.",
        source_observation=dict(previous_plan_sha256=sha(OLD / "plan.json"),
            previous_summary_sha256=sha(OLD / "analysis/summary.json"),
            interpretation="The previous fine-setting capture stopped outside the goal; do not adopt that setting or change historical failures. The new experiment holds reference physics fixed."),
        fixed_screenshot_selection=["first_saved_camera_anchor", "launch_to_normal_pid", "normal_pid_to_zero_pedal_coast",
            "zero_pedal_coast_to_terminal_feedback", "zero_pedal_coast_to_service_brake_hold", "first_full_hold_brake_readback", "last_saved_camera_anchor"],
        continuation_policy="Proceed to feedback only after coast passes execution, scalar, pixel, initialization, ACK and independently replayed control protocol checks. Original future failures stay visible and do not authorize admission. Any operational failure blocks the remaining case; no retry.",
        comparison_criteria=dict(
            primary="Report completion under unchanged goal/speed/dwell/tail/timeouts, intervention coverage and all original native/future gate results. One pair is condition-specific and cannot establish repeatability or all-map capability.",
            secondary="Compare the full physical prefix before the first feedback command, then report all measured future count/rate/max changes and raw terminal movement without retiming or favorable clipping.",
            incomplete="No comparative verdict if either case is blocked or fails.",
            admission="No diagnostic result automatically admits data, starts training or promotes a learned model."),
        readiness_policy=old["readiness_policy"],
        operational_scope=dict(local_owned_carla_only=True, port_range=[2100, 2101, 2102],
            foreign_process_signals=False, remote_commands=False, test_payload_reads=False, no_installs=True,
            cleanup_notice="Use original owned-process cleanup; no independent eleven-field settings-restoration claim."),
        training_data_approved=False, learned_model_control=False, vehicle_control_approved=False,
        dataset_export=False, target_retiming=False, gate_changes=False, research_only=True, development_only=True,
        old_artifacts_modified=False, full_model_inference=False, pose_or_velocity_reset=False)
    from scripts.e2e.summarize_carla_terminal_approach import validate_plan
    validate_plan(plan)
    with (BASE / "plan.json").open("x") as stream:
        json.dump(plan, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(dict(plan=str(BASE / "plan.json"), sha256=sha(BASE / "plan.json"), cases=2)))


if __name__ == "__main__":
    main()
