#!/usr/bin/env python3
"""HH_260906 - Freeze four fresh numerical-setting trials before launch, with no change to controller, sensors or original admission gates."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[4]
OLD = ROOT / "artifacts/training/2026-09-14/hold_brake_ramp_abba_v7"


def sha(path):
    assert path.is_file() and not any(p.is_symlink() for p in (path, *path.parents)), path
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    assert sha(OLD / "plan.json") == "a7708081681efb7dddf9fc52e547a0950b500dcb2c36f5be9f1deaa2402d0a48"
    old = json.loads((OLD / "plan.json").read_text())
    for category in ("source_sha256", "input_sha256", "audit_math_sha256", "orchestration_sha256"):
        for name, pin in old[category].items():
            assert sha(ROOT / name) == pin, name
    cases = []
    for name, arm in (("01_reference_A1", "reference"), ("02_fine_B1", "fine"),
                      ("03_fine_B2", "fine"), ("04_reference_A2", "reference")):
        command = old["cases"][0]["command"].copy()
        command[2] = str((BASE / name).relative_to(ROOT))
        command[command.index("--physics-substep-profile") + 1] = "reference_10ms" if arm == "reference" else "fine_5ms"
        assert command[command.index("--hold-brake-mode") + 1] == "immediate"
        assert not (BASE / name).exists()
        cases.append(dict(id=name, arm=arm, command=command, maximum_attempts=1))
    source_pins = dict(old["source_sha256"])
    helper = "scripts/e2e/carla_physics_substeps.py"
    source_pins[helper] = sha(ROOT / helper)
    orchestration = {"scripts/e2e/probe_carla_port_readiness.py": sha(ROOT / "scripts/e2e/probe_carla_port_readiness.py")}
    for name in ("run_case.py", "render_evidence.py"):
        orchestration[str((BASE / name).relative_to(ROOT))] = sha(BASE / name)
    analysis = {name: sha(ROOT / name) for name in (
        "scripts/e2e/summarize_carla_hold_substep_abba.py",
        "scripts/e2e/summarize_carla_hold_brake_ramp_abba.py",
        "scripts/e2e/summarize_carla_low_speed_steering_abba.py")}
    keys = ("control_transport", "agent_initialization", "target_speed_kmh", "physics_hz", "camera_hz",
            "stationary_warmup_seconds", "required_measured_stop_dwell_seconds", "stationary_tail_seconds",
            "maximum_total_simulation_seconds", "collector_wall_timeout_seconds", "quality", "map", "scenario",
            "vehicle_type", "wheelbase_m", "spawn_z_offset_m", "seed", "weather", "fixed_screenshot_selection", "screenshot_fallback")
    plan = {key: old[key] for key in keys}
    plan.update(schema="carla.hold_substep_abba_plan.v1", comment=__doc__,
        declared_at_utc=datetime.now(timezone.utc).isoformat(), campaign_id="town07_immediate_hold_substep_abba_v8",
        cases=cases, maximum_attempts_per_revision=4, automatic_retry=False,
        source_sha256=source_pins, input_sha256=old["input_sha256"], audit_math_sha256=old["audit_math_sha256"],
        analysis_sha256=analysis, orchestration_sha256=orchestration,
        protocol_configuration=old["protocol_configuration_by_arm"]["immediate"],
        physics_profile_by_arm={"reference": "reference_10ms", "fine": "fine_5ms"},
        required_original_substeps=dict(substepping=True, max_substep_delta_time=.01, max_substeps=10),
        requested_max_substep_delta_time_by_arm={"reference": .01, "fine": .005},
        fixed_max_substeps=10, explicit_settings_readback_required=True,
        changed_factor="Only max_substep_delta_time .01 vs .005; fixed 20 Hz state/control, 10 Hz cameras, same immediate hold and zero low-speed steering.",
        independent_variable_notice="Maximum numerical setting is not measured actual internal substeps or physical accuracy/actuation timing proof.",
        source_observation=dict(campaign_plan_sha256=sha(OLD / "plan.json"),
            sealed_summary_sha256=sha(OLD / "analysis/summary.json"),
            first_hold_actor_forward_displacement_divided_by_entry_velocity_seconds=.01000010949481274,
            interpretation="A measured ratio motivates sensitivity analysis; not a substep observation or predicted outcome."),
        continuation_policy="Proceed only after all previous cases pass execution, scalar, pixel, initialization, ACK and unchanged-control protocol checks. Original future failures are preserved and do not authorize admission. Any operational failure blocks remaining cases; no retry.",
        comparison_criteria=dict(
            primary="Both fine repeats must reduce full original curvature-failing anchor count AND rate against BOTH reference repeats, with no other original gate count/rate regression and all four operational diagnostics complete.",
            secondary="Report all original maxima, repeated physical traces, failure endpoints, first normal hold interval signed displacement/velocity proxies, and pose/velocity residuals; never omit launch or tail.",
            incomplete="No four-case comparative verdict if any case is blocked or fails.",
            admission="No diagnostic result automatically admits data or promotes a learned model."),
        readiness_policy=old["readiness_policy"],
        operational_scope=dict(local_owned_carla_only=True, port_range=[2100,2101,2102],
            foreign_process_signals=False, remote_commands=False, test_payload_reads=False, no_installs=True,
            cleanup_notice="Use original owned-process cleanup; no independent eleven-field settings-restoration claim."),
        training_data_approved=False, learned_model_control=False, vehicle_control_approved=False,
        dataset_export=False, target_retiming=False, gate_changes=False, research_only=True, development_only=True,
        old_artifacts_modified=False, full_model_inference=False, pose_or_velocity_reset=False)
    with (BASE / "plan.json").open("x") as stream:
        json.dump(plan, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(dict(plan=str(BASE / "plan.json"), sha256=sha(BASE / "plan.json"), cases=4)))


if __name__ == "__main__":
    main()
