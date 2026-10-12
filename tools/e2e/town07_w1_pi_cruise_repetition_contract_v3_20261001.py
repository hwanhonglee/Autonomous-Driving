"""HH_261001 - Predeclare three fixed 45-second W1 PI cruise research repetitions."""

from datetime import datetime, timezone
from pathlib import Path
import shutil

from scripts.e2e import town07_moving_brake_ab_contract_v2_20261001 as assets_v2


ROOT = assets_v2.ROOT
RUN_ROOT = ROOT / "storage/experiments/validation/2026-10-01/town07_w1_pi_cruise_repetition_v3"
PLAN = RUN_ROOT.with_name("town07_w1_pi_cruise_repetition_plan_v3.json")
SCHEMA = "portable_e2e.town07_w1_pi_cruise_repetition.20261001.v3"
MODULE = "scripts.e2e.research_town07_w1_pi_cruise_repetition_v3_20261001"
FLOOR, CLOSING_RESERVE = 5 * 1024**3, 16 * 1024**2
TRIAL_RESERVE, SHARED_RESERVE, SAMPLE_LIMIT = 512 * 1024**2, 64 * 1024**2, 128 * 1024**2
BUDGET, SAMPLE_EVERY, PORT = 45., 20, 2440
require, digest, read_json, write_new = (
    assets_v2.require, assets_v2.digest, assets_v2.read_json, assets_v2.write_new)

# HH_261001 - Explicit imported dependency closure, including lazy collector helpers; never a learning directory glob.
SOURCE_FILES = (
    "learning/portable_e2e/__init__.py",
    "learning/portable_e2e/contract.py",
    "learning/portable_e2e/dataset.py",
    "learning/portable_e2e/evaluate.py",
    "learning/portable_e2e/longitudinal_pi_research_20261001.py",
    "learning/portable_e2e/losses.py",
    "learning/portable_e2e/model.py",
    "learning/portable_e2e/runtime.py",
    "learning/portable_e2e/runtime_contract.py",
    "learning/portable_e2e/runtime_weight_bundle.py",
    "learning/portable_e2e/simulation_research.py",
    "learning/portable_e2e/stop_primitive_research.py",
    "learning/portable_e2e/torch_dataset.py",
    "learning/portable_e2e/train.py",
    "learning/portable_e2e/visualize.py",
    "learning/portable_e2e/workspace_paths.py",
    "learning/portable_e2e/config/common_10hz_v1.contract.json",
    "tools/canonical_workspace.py",
    "tools/workspace_python.py",
    "tools/e2e/audit_carla_acknowledged_control.py",
    "tools/e2e/audit_carla_comfortable_v3_trial.py",
    "tools/e2e/audit_future_actor_lane_containment.py",
    "tools/e2e/audit_portable_target_feasibility.py",
    "tools/e2e/audit_town13_source_editor_preflight.py",
    "tools/e2e/capture_exact_simulation_inputs_20261001.py",
    "tools/e2e/carla_goal_stop_profile.py",
    "tools/e2e/carla_moving_capture.py",
    "tools/e2e/carla_physics_substeps.py",
    "tools/e2e/carla_wall_timing.py",
    "tools/e2e/collect_ah_simulation_research.py",
    "tools/e2e/collect_carla_vad_expert.py",
    "tools/e2e/env.sh",
    "tools/e2e/inventory_carla_training_maps.py",
    "tools/e2e/physical_straight_geometry.py",
    "tools/e2e/physical_turn_geometry.py",
    "tools/e2e/prepare_carla_common10_dataset.py",
    "tools/e2e/prepare_carla_expert_route_catalog.py",
    "tools/e2e/prepare_ctrack_isolated_probe_20261001.py",
    "tools/e2e/qualify_carla_moving_common10.py",
    "tools/e2e/run_carla_map.sh",
    "tools/e2e/run_owned_ah_town07_research_20261001.py",
    "tools/e2e/run_owned_ctrack_isolated_probe_20261001.py",
    "tools/e2e/run_owned_town13_route_catalog.py",
    "tools/e2e/serialized_custom_turn_geometry.py",
    "tools/e2e/summarize_carla_goal_stop_trials.py",
    "tools/e2e/town07_moving_brake_ab_contract_v2_20261001.py",
    "tools/e2e/town07_w1_pi_control_20261001.py",
    "tools/e2e/town07_w1_pi_cruise_repetition_contract_v3_20261001.py",
    "tools/e2e/research_town07_w1_pi_cruise_repetition_v3_20261001.py",
)


def trial_specs():
    return tuple(dict(index=i, trial_id=f"B{i+1}_pi_only_45s", arm="B_pi_only",
                      output=str(RUN_ROOT / f"B{i+1}_pi_only_45s")) for i in range(3))


def trial(index):
    require(type(index) is int and index in (0, 1, 2), "one fixed repetition index 0, 1 or 2 required")
    return trial_specs()[index]


def output(index):
    return Path(trial(index)["output"])


def fixed_contract():
    value = assets_v2.fixed_contract()
    value.update(schema=SCHEMA, trial_order=list(trial_specs()), attempts_per_arm=3,
        intervention="Frozen W1 B_pi_only controller repeated; model, selected path/speed, steering, stop and emergency unchanged",
        interpretation="Three same-TRAIN-route low-speed cruise reproducibility research repetitions, NOT full RTE-01, route-goal success or generalization",
        independent_replication=False, controller_only_comparison=False,
        integral_added=True, proportional_gain=.7, integral_gain=.2, integral_limit=1.,
        control_timestep_s=.1, inference_deadline_s=.1,
        total_capture_seconds=BUDGET, expected_native_states=900, expected_camera_bundles=450,
        sparse_exact_input_policy_call_indices="0,20,40,...", sparse_exact_inputs_every=SAMPLE_EVERY,
        sample_policy_revision="NEW storage-only every-20th inference; prior every-10th owners and evidence unchanged",
        maximum_exact_npz_samples=23, actual_prior_npz_bytes=4150804,
        maximum_expected_exact_npz_bytes=23 * 4150804,
        pixel_and_built_tensor_hashes_all_inferences=True,
        hash_observer_cost_included_in_original_policy_deadline=True,
        end_to_end_realtime_fps_approved=False,
        original_port_bind_check_natural_release_wait_s=75., port_release_poll_seconds=.2,
        socket_reuse_enabled=False, forced_socket_release=False,
        per_arm_reserved_bytes=TRIAL_RESERVE, sample_directory_maximum_bytes=SAMPLE_LIMIT,
        shared_safety_reserved_bytes=SHARED_RESERVE,
        source_closure_policy="explicit imported dependency-only list; no source directory globs",
        prospective_motion_screen=dict(
            denominator="all three predeclared attempts, including capture, motion, storage and ownership failures",
            interval="first observation with prior learned ACK through the final captured driving observation; no selected end window",
            net_progress_minimum_m=90., stationary_threshold_mps=.05,
            unintended_stationary_dwell_limit_s=1., cte_p95_maximum_m=.35,
            cte_absolute_maximum_m=.75, maximum_measured_actor_planar_speed_mps=2.7,
            collisions_maximum=0, lane_invasions_maximum=0,
            body_or_swept_lane_clearance_below_point_one_m_maximum=0,
            all_native_held_acknowledgements_required=True,
            learned_rejected_commands_maximum=0, emergency_commands_maximum=0,
            explicit_stop_candidates_maximum=0, stop_candidate_indices=list(range(6, 12)),
            model_desired_stop_threshold_mps=.05,
            inference_control_deadline_seconds=.1,
            all_three_must_pass=True, full_feature_approval=False))
    return value


def source_pins():
    require(digest(assets_v2.FREEZE) == assets_v2.FREEZE_SHA, "old runtime freeze bytes changed")
    old = read_json(assets_v2.FREEZE)
    require(old.get("schema") == "portable_e2e.directional_replay_runtime_source_freeze.v1",
            "old runtime freeze schema changed")
    pins = {}
    require(len(set(SOURCE_FILES)) == len(SOURCE_FILES), "explicit dependency list contains duplicates")
    for name in SOURCE_FILES:
        actual = digest(ROOT / name)
        if name in old["source_sha256"]:
            require(actual == old["source_sha256"][name], "frozen dependency changed: " + name)
        pins[name] = actual
    pins[str(assets_v2.FREEZE.relative_to(ROOT))] = assets_v2.FREEZE_SHA
    return pins


def disk_check(remaining=3, free=None):
    require(type(remaining) is int and remaining in (1, 2, 3), "remaining fixed repetitions required")
    available = shutil.disk_usage(ROOT).free if free is None else free
    needed = FLOOR + remaining * TRIAL_RESERVE + SHARED_RESERVE
    require(type(available) is int and available >= needed, f"need {needed} bytes for fixed repetitions; have {available}")
    return dict(free_bytes=available, required_bytes=needed, floor_bytes=FLOOR,
                remaining_repetitions=remaining, per_trial_reserved_bytes=TRIAL_RESERVE,
                exact_sample_allowance_bytes=SAMPLE_LIMIT, shared_reserved_bytes=SHARED_RESERVE)


def build_plan():
    require(not any(output(i).exists() or output(i).is_symlink() for i in range(3)),
            "declaration must precede all three physical attempts")
    assets = assets_v2.assets()
    require(all(digest(name) == sha for name, sha in assets.items()), "old owner pinned assets changed")
    return dict(fixed_contract(), declared_at_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=source_pins(), asset_sha256=assets,
        external_agent_sha256={str(path): digest(path) for path in assets_v2.AGENT_SOURCES})


def verify_plan(value, expected):
    require(isinstance(expected, str) and len(expected) == 64
            and all(c in "0123456789abcdef" for c in expected), "exact declaration SHA required")
    require(digest(PLAN) == expected, "declared plan bytes changed")
    fixed = fixed_contract()
    require(set(value) == set(fixed) | {"declared_at_utc", "source_sha256", "asset_sha256", "external_agent_sha256"},
            "declared plan fields changed")
    require(all(value[key] == item for key, item in fixed.items()), "fixed repetition contract changed")
    stamp = datetime.fromisoformat(value["declared_at_utc"])
    require(stamp.tzinfo is not None and stamp.utcoffset().total_seconds() == 0
            and stamp <= datetime.now(timezone.utc), "prior UTC declaration required")
    require(value["source_sha256"] == source_pins(), "declared dependency closure changed")
    require(value["asset_sha256"] == assets_v2.assets(), "old asset pins changed")
    require(set(value["external_agent_sha256"]) == {str(path) for path in assets_v2.AGENT_SOURCES},
            "external agent source closure changed")
    for name, sha in dict(value["asset_sha256"], **value["external_agent_sha256"]).items():
        require(digest(name) == sha, "pinned asset or agent changed: " + name)
    return value
